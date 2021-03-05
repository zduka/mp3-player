/** Library for working with neopixels. Comes with the megatinycore and does not require any additional arduino setup. 
 */
#include <tinyNeoPixel_Static.h>

#include "inputs.h"

/** Chip Pinout

               -- VDD   GND --
     VCC_SENSE -- (00) PA4   PA3 (16) --
    HEADPHONES -- (01) PA5   PA2 (15) -- MIC
      DCDC_PWR -- (02) PA6   PA1 (14) -- 
     AUDIO_SRC -- (03) PA7   PA0 (17) -- UPDI
     AUDIO_ADC -- (04) PB5   PC3 (13) -- CTRL_A
         VOL_B -- (05) PB4   PC2 (12) -- CTRL_BTN
         VOL_A -- (06) PB3   PC1 (11) -- NEOPIXEL
       VOL_BTN -- (07) PB2   PC0 (10) -- CTRL_B
           SDA -- (08) PB1   PB0 (09) -- SCL

    - monitor input voltage
    
 */

#define DCDC_PWR 2
#define NEOPIXEL 11
#define CTRL_A 13
#define CTRL_B 10
#define CTRL_BTN 12
#define VOL_A 6
#define VOL_B 5
#define VOL_BTN 7
/** The control (top) and volume (down) knobs (rotary encoders and buttons). 
 */
RotaryEncoder ctrl{CTRL_A, CTRL_B, 255};
Button        ctrlBtn{CTRL_BTN};
RotaryEncoder vol{VOL_A, VOL_B, 15};
Button        volBtn{VOL_BTN};

/** Manages the 3v3 and 5v rails and DCDC converters. 
 */
class DCDCPower {
public:
    /** Starts the manager and makes sure to turn the dc rails. 
     */
    DCDCPower() {
        pinMode(DCDC_PWR, OUTPUT);
        digitalWrite(DCDC_PWR, LOW);
    }

    /** Requests the power to be enabled. 
     */
    void request() {
        if (++clients_ == 1) {
            digitalWrite(DCDC_PWR, HIGH);
            delay(50);            
        }
    }

    void release() {
        if (clients_ > 0) {
            if (--clients_ == 0)
                digitalWrite(DCDC_PWR, LOW);
        }
    }

    bool state() const {
        return  clients_ > 0;
    }

private:
    volatile uint8_t clients_;
};

DCDCPower power;


/** The neopixel strip and its features. 
 */
class NeopixelStrip {
public:
    NeopixelStrip():
        leds_{8, NEOPIXEL, NEO_GRB + NEO_KHZ800, pixels_} {
        pinMode(NEOPIXEL,OUTPUT);
    }

    void enable() {
        power.request();
        setAll(0,0,0);
    }

    void disable() {
        power.release();
    }

    
    void setAll(uint8_t r, uint8_t g, uint8_t b) {
        for (int i = 0; i < 8; ++i) {
            leds_.setPixelColor(i, r, g, b);
        }
        leds_.show();
    }

private:
    byte pixels_[8 * 3];
    tinyNeoPixel leds_;// = tinyNeoPixel(2, NEOPIXEL, NEO_GRB + NEO_KHZ800, pixels);
};

NeopixelStrip neopixelStrip;

void vol_changed() {
    vol.poll();
}

void vol_btn_changed() {
    volBtn.poll();
}

void ctrl_changed() {
    ctrl.poll();
}

void ctrl_btn_changed() {
    ctrlBtn.poll();
}


extern "C" void RTC_PIT_vect(void) __attribute__((signal));


/** The MP3 Player Device Controller

    # RTC

    The internal RTC in the attiny is used to keep a semi-precise track of time. When the chip is on, a tick is generated every 1/32th of second so that it can be used to drive neopixels, etc. When the device sleeps, a one second tick is used to save battery and only the time is updated and alarm checked. 
 */
class Player {
public:

    Player() {
      rtcEnable();
    }

    /** Returns true if there was an RTC tick since the last call to the function. 

        An RTC tick occurs every 1/32th of a second. 
     */
    bool rtcTick() const {
      if (status_.rtcTick) {
        status_.rtcTick = 0;
        return true;
      } else {
        return false;
      }
    }

    /** Returns true if there was a second tick since the last call to the function. 
     */
    bool rtcTickSecond() const {
      if (status_.rtcTickSecond) {
        status_.rtcTickSecond = 0;
        return true;
      } else {
        return false;
      }
    }
private:

    void rtcEnable() {
        RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
        RTC.PITINTCTRL |= RTC_PI_bm;
        RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc + RTC_PITEN_bm;
    }

    void rtcDisable() {
        RTC.PITCTRLA &= ~ RTC_PITEN_bm;
    }

    void doRtcTick() {
      status_.rtcTick = 1;
      if (++rtcTick_ == 32) {
          rtcTick_ = 0;
          doRtcTickSecond();
      }
    }

    void doRtcTickSecond() {
      status_.rtcTickSecond = 1;
      if (++s_ < 60)
          return;
      s_ = 0;
      if (++m_ < 60)
          return;
      m_ = 0;
      if (++h_ < 24) 
          return;
      h_ = 0;
      if (++day_ <= daysInMonth())
          return;
      day_ = 1;
      if (++month_ <= 12)
          return;
      month_ = 1;
      ++year_;
    }

    uint8_t daysInMonth() {
      switch (month_) {
        case 1: // Jan
        case 3: // Mar
        case 5: // May
        case 7: // Jul
        case 8: // Aug
        case 10: // Oct
        case 12: // Dec
            return 31;
        case 2 : // Feb
            // I'm ignoring the every 100 years leap year skip as the code will hopefully not be around for that long:)
            return (year_ % 4 == 0) ? 29 : 28;
        case 4:
        case 6:
        case 9:
        case 11:
        default: // whatever
            return 30;
      }
    }

    /** \name Device status. 

        Contains the device status information, such as power levels, etc. Readonly from the I2C.
     
     */
    /** Power information. 
     */
    struct {
        unsigned charging : 1;
        /** Shows the current input voltage. 

            Is constructed by reading the input voltage in scale 0..255 corresponding to 0..5v and then discarding the most signifficant byte as under normal conditions, this will always be 1. 

            127 == 5v from external source (and likely charging)
            86  == 4.2v, a fully charged li-ion battery
            60  == 3.7v, nominal value for 
            40  == 3.3v, value at which the low battery warning is given
            25  == 3v, value at which attiny goes immediately to deepsleep again
         */
        unsigned voltage : 7;
    } power_;
    /** \name Audo Settings
     */
    struct {
        /** Current volume setting. 
         */
        unsigned volume : 4;
        /** Determines the audio source. 

            0 = esp8266 mp3 player
            1 = RDA5807 FM radio
         */
        unsigned source : 1;
        /** If 1, headphones are inserted, if 0, speaker will be used instead. Can only be read, and writes will be masked.
         */
        unsigned headphones : 1;
        
        
       
    } audio_;
    /** \name Real time clock and alarms. 

        Can be set via I2C. Contains the current clock maintained by the chip as well as the alarm settings. The alarm day can be number from 1 to 7 for days of the week, starting from Sunday. If the alarmDay is set to 0, the alarm is not active. 
     */
    //@{
    volatile uint8_t year_ = 0;
    volatile uint8_t month_ = 0;
    volatile uint8_t day_ = 0;
    volatile uint8_t h_ = 0;
    volatile uint8_t m_ = 0;
    volatile uint8_t s_ = 0;
    volatile uint8_t alarmDay_ = 0; 
    volatile uint8_t alarmH_ = 0;
    volatile uint8_t alarmM_ = 0;
    //@}
    /** \name Accent Color

        Specifies the accent color of the device, which is used for the neopixel stripe in default settings. Can be set via I2C. 
     */
    //@{
    volatile uint8_t accentRed_ = 255;
    volatile uint8_t accentGreen_ = 255;
    volatile uint8_t accentBlue_ = 255;
    //@}








    /** RTC ticks counter, there are 32 ticks in one second. 
     */
    volatile uint8_t rtcTick_ = 0;

    mutable volatile struct {
        unsigned irq : 1;
        unsigned dcdcPower : 1;
        unsigned rtcTick : 1;
        unsigned rtcTickSecond : 1;
    } status_;

    friend void ::RTC_PIT_vect();

}; // Player

Player player; // the player singleton

ISR(RTC_PIT_vect) {
   RTC.PITINTFLAGS = RTC_PI_bm;
   player.doRtcTick();
}






void setup() {
    Serial.swap();
    Serial.begin(9600);
    Serial.println("Hello world");

    
    vol.setInterrupt(vol_changed);
    ctrl.setInterrupt(ctrl_changed);
    volBtn.setInterrupt(vol_btn_changed);
    ctrlBtn.setInterrupt(ctrl_btn_changed);
    
    // quick blink of the strip
    neopixelStrip.enable();
    neopixelStrip.setAll(0, 128, 0);
    delay(100);
    neopixelStrip.setAll(0, 0, 0);

    
    
    

}



/** During the loop, the following things must be checked:

    - determine the input voltage by reading the VCC_SENSE (low)
    
 */
void loop() {
    static bool x = false;
    if (volBtn.changed() && volBtn.pressed())
      vol.setValue(0);
    if (ctrlBtn.changed() && ctrlBtn.pressed()) 
      ctrl.setValue(0);
    if (vol.changed() || ctrl.changed()) 
      neopixelStrip.setAll(vol.value() * 16, ctrl.value(), 0);
    if (player.rtcTickSecond()) {
      if (x)
          neopixelStrip.setAll(64,0,0);
      else
          neopixelStrip.setAll(0,0,64);
      x = !x;  
    }
}
