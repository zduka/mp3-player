/** Library for working with neopixels. Comes with the megatinycore and does not require any additional arduino setup. 
 */
#include <tinyNeoPixel_Static.h>

#include "state.h"
#include "inputs.h"

/** Chip Pinout

               -- VDD   GND --
     VCC_SENSE -- (00) PA4   PA3 (16) -- AVR_IRQ
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
#define AVR_IRQ 16

/** The control (top) and volume (down) knobs (rotary encoders and buttons). 
 */
RotaryEncoder ctrl{CTRL_A, CTRL_B, 255};
Button        ctrlBtn{CTRL_BTN};
RotaryEncoder vol{VOL_A, VOL_B, 15};
Button        volBtn{VOL_BTN};

/** The neopixel strip and its features. 
 */
class NeopixelStrip {
public:
    NeopixelStrip():
        leds_{8, NEOPIXEL, NEO_GRB + NEO_KHZ800, pixels_} {
        pinMode(NEOPIXEL,OUTPUT);
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

extern "C" void RTC_PIT_vect(void) __attribute__((signal));
extern "C" void TWI0_TWIS_vect(void) __attribute__((signal));


/** The MP3 Player Device Controller

    # RTC

    The internal RTC in the attiny is used to keep a semi-precise track of time. When the chip is on, a tick is generated every 1/32th of second so that it can be used to drive neopixels, etc. When the device sleeps, a one second tick is used to save battery and only the time is updated and alarm checked. 
 */
class Player {
public:

    Player() {
        // enable the AVR_IRQ as input
        pinMode(AVR_IRQ, INPUT);
        // disable 3v and 5v rails
        pinMode(DCDC_PWR, OUTPUT);
        digitalWrite(DCDC_PWR, LOW);
        // enable real-time clock
        RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; // select internal oscillator
        RTC.PITINTCTRL |= RTC_PI_bm; // enable the interrupt
        RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc + RTC_PITEN_bm; // enable PTI and set the tick to 1/32th of second
        // enable I2C slave
        TWI0.SADDR = AVR_I2C_ADDRESS; // set address
        TWI0.SCTRLA = TWI_DIEN_bm + TWI_APIEN_bm + TWI_PIEN_bm + TWI_ENABLE_bm; // enable address, data and stop interrupts and enable the i2c slave
    }

    /** Returns true if there was an RTC tick since the last call to the function. 

        An RTC tick occurs every 1/32th of a second. 
     */
    bool rtcTick() const {
        if (rtcTick_.tick) {
            rtcTick_.tick = 0;
            return true;
        } else {
            return false;
        }
    }

    /** Returns true if there was a second tick since the last call to the function. 
     */
    bool rtcTickSecond() const {
        if (rtcTick_.tickSecond) {
            rtcTick_.tickSecond = 0;
            return true;
        } else {
            return false;
        }
    }

    void requestPowerOn() {
        if (++powerRequests_ == 1) {
            digitalWrite(DCDC_PWR, HIGH);
            delay(50);
        }
    }

    void releasePowerOn() {
        if (powerRequests_ > 0) {
            if (--powerRequests_ == 0) {
                digitalWrite(DCDC_PWR, LOW);
            }
        }
    }

    void setVolume(uint8_t value) {
        if (value != audio_.volume) {
            audio_.volume = value;
            setIrq();
        }
    }

    
private:

    /** Sets the IRQ to ping the esp8266. 
     */
    bool setIrq() {
        if (status_.irq == 0) {
            status_.irq = 1;
            pinMode(AVR_IRQ,OUTPUT);
            digitalWrite(AVR_IRQ, LOW);
            return true;
        } else {
            return false;
        }
    }

    bool clearIrq() {
        if (status_.irq == 1) {
            status_.irq = 0;
            pinMode(AVR_IRQ, INPUT);
            return true;
        } else {
            return false;
        }
    }


    void rtcDisable() {
        RTC.PITCTRLA &= ~ RTC_PITEN_bm;
    }

    void doRtcTick() {
        rtcTick_.tick = 1;
        if (++rtcTick_.counter == 0)
            doRtcTickSecond();
    }

    void doRtcTickSecond() {
        rtcTick_.tickSecond = 1;
        if (++clock_.s < 60)
            return;
        clock_.s = 0;
        if (++clock_.m < 60)
            return;
        clock_.m = 0;
        if (++clock_.h < 24) 
            return;
        clock_.h = 0;
        if (++clock_.day <= daysInMonth())
            return;
        clock_.day = 1;
        if (++clock_.month <= 12)
            return;
        clock_.month = 1;
        ++clock_.year;
    }

    uint8_t daysInMonth() {
      switch (clock_.month) {
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
            return (clock_.year % 4 == 0) ? 29 : 28;
        case 4:
        case 6:
        case 9:
        case 11:
        default: // whatever
            return 30;
      }
    }

    /** The device control registers:

        size kind
        1    power
        1    audio
        1    year
        1    month
        1    day

   

     */
    
    /** \name Device status. 

        Contains the device status information, such as power levels, etc. Readonly from the I2C.
     
     */
    Status status_;
    Power power_;
    Audio audio_;
    Clock clock_;
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
    mutable volatile struct {
        unsigned tick : 1;
        unsigned tickSecond : 1;
        unsigned counter : 5; // 0..31
    } rtcTick_;
    

    volatile uint8_t powerRequests_ = 0;

    friend void ::RTC_PIT_vect();
    friend void ::TWI0_TWIS_vect();

}; // Player

Player player; // the player singleton

/** Real-time clock tick interrupt. 
    
    When running, happens every 1/32th of a second. When sleeping, should happen every second only. 
 */
ISR(RTC_PIT_vect) {
   RTC.PITINTFLAGS = RTC_PI_bm;
   player.doRtcTick();
}

/** I2C slave interrupt vector. 
    
 */
ISR(TWI0_TWIS_vect) {
    switch (TWI0.SSTATUS & 0b11000011) {
        /* Master read start. 
         */
        case 0b01000011: {
            player.clearIrq();

            break;
        }
        /* Master write start.
         */
        case 0b01000001: {
            player.clearIrq();

            break;
        }
        /* Stop condition has been received. 
         */
        case 0b01000010:
        case 0b01000000: {
            break;
        }
        /* Byte was read by master. 
         */
        case 0b10000010: {
            break;
        }
        /* Byte was written by master. 
         */
        case 0b10000000: {
            break;
        }
        /* We really don't care about the restof the states in this simple implementation.
         */
        default:
            break;
    }
}



void vol_changed() {
    vol.poll();
    player.setVolume(vol.value());
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






void setup() {
    /*
    Serial.swap();
    Serial.begin(9600);
    Serial.println("Hello world");
    */

    
    vol.setInterrupt(vol_changed);
    ctrl.setInterrupt(ctrl_changed);
    volBtn.setInterrupt(vol_btn_changed);
    ctrlBtn.setInterrupt(ctrl_btn_changed);

    player.requestPowerOn();
    
    // quick blink of the strip
    //    neopixelStrip.enable();
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
            neopixelStrip.setAll(16,0,0);
        else
            neopixelStrip.setAll(0,0,16);
        x = !x;  
    }
}
