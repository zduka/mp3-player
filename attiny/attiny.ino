#include "Wire.h"

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

volatile uint8_t g = 5;


#define DCDC_PWR 2
#define NEOPIXEL 11
#define CTRL_A 13
#define CTRL_B 10
#define CTRL_BTN 12
#define VOL_A 6
#define VOL_B 5
#define VOL_BTN 7
#define AVR_IRQ 16
#define AUDIO_SRC 3

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

/** The MP3 Player Device Controller

    # RTC

    The internal RTC in the attiny is used to keep a semi-precise track of time. When the chip is on, a tick is generated every 1/32th of second so that it can be used to drive neopixels, etc. When the device sleeps, a one second tick is used to save battery and only the time is updated and alarm checked. 
 */
class AVRPlayer {
public:

    AVRPlayer() {
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
        Wire.begin(AVR_I2C_ADDRESS);
        Wire.onRequest(I2CSendEvent);
        Wire.onReceive(I2CReceiveEvent);
        // set audio source to esp8266
        pinMode(AUDIO_SRC, OUTPUT);
        digitalWrite(AUDIO_SRC, LOW);
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
            state_.state_ |= State::STATE_DCDC_POWER; 
            delay(50);
        }
    }

    void releasePowerOn() {
        if (powerRequests_ > 0) {
            if (--powerRequests_ == 0) {
                digitalWrite(DCDC_PWR, LOW);
                state_.state_ &= ~State::STATE_DCDC_POWER;
            }
        }
    }

    void setVolume(uint8_t value) {
        if (value != state_.volume()) {
            state_.setVolume(value);
            setIrq();
        }
    }

    void setControl(uint16_t value) {
        if (value != state_.control()) {
            state_.setControl(value);
            setIrq();
        }
    }

    void enterMP3Mode() {
        
    }

    /** Enters the radio mode. 
     */
    void enterRadioMode(uint16_t controlValue, uint16_t controlMax) {
        ctrl.setMaxValue(controlMax);
        ctrl.setValue(controlValue);
        digitalWrite(AUDIO_SRC, HIGH);
        state_.setAudioSrc(State::AudioSrc::Radio);
        state_.setMode(State::Mode::Radio);
    }

private:

    /** Sets the IRQ to ping the esp8266. 
     */
    bool setIrq() {
        if (!state_.irq()) {
            state_.events_ |= State::EVENT_IRQ;
            pinMode(AVR_IRQ,OUTPUT);
            digitalWrite(AVR_IRQ, LOW);
            return true;
        } else {
            return false;
        }
    }

    void clearIrq() {
        if (state_.events_ & State::EVENT_IRQ) {
            state_.events_ = 0;
            pinMode(AVR_IRQ, INPUT);
        }
    }

    void rtcDisable() {
        RTC.PITCTRLA &= ~ RTC_PITEN_bm;
    }

    void doRtcTick() {
        rtcTick_.tick = 1;
        if (++rtcTick_.counter == 0) {
            rtcTick_.tickSecond = 1;
            clock_.secondTick();
        }
    }

    /** Device state. 
     */
    volatile State state_;
    volatile Clock clock_;
    
    
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
    volatile uint8_t * i2cPtr_ = nullptr;

    friend void ::RTC_PIT_vect();

    static void I2CSendEvent();
    static void I2CReceiveEvent(int numBytes);

    uint8_t cmdBuffer_[32];

}; // Player


AVRPlayer player; // the player singleton

/** Real-time clock tick interrupt. 
    
    When running, happens every 1/32th of a second. When sleeping, should happen every second only. 
 */
ISR(RTC_PIT_vect) {
   RTC.PITINTFLAGS = RTC_PI_bm;
   player.doRtcTick();
}


void AVRPlayer::I2CSendEvent() {
    // casting away the volatile-ness is ok here 
    Wire.write(pointer_cast<uint8_t*>(const_cast<State*>(& player.state_)), sizeof (State));
    // clear the irq and events once we have given them to master
    player.clearIrq();
    player.state_.events_ = 0;
}

void AVRPlayer::I2CReceiveEvent(int numBytes) {
    Wire.readBytes(player.cmdBuffer_, numBytes);
    switch (player.cmdBuffer_[0]) {
    case Command::EnterMP3Mode::Id: {
        player.enterMP3Mode();
        break;
    }
    case Command::EnterRadioMode::Id: {
        Command::EnterRadioMode * cmd = pointer_cast<Command::EnterRadioMode*>(& player.cmdBuffer_ + 1);
        player.enterRadioMode(cmd->controlValue, cmd->controlMaxValue);
        break;
    }
    case Command::SetVolume::Id: {
        Command::SetVolume * cmd = pointer_cast<Command::SetVolume*>(& player.cmdBuffer_ + 1);
        player.state_.setVolume(cmd->value, /* recordChange */ false);
        vol.setValue(cmd->value);
        break;
    }
    case Command::SetControl::Id: {
        Command::SetControl * cmd = pointer_cast<Command::SetControl*>(& player.cmdBuffer_ + 1);
        break;
    }
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
    player.setControl(ctrl.value());
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
            neopixelStrip.setAll(vol.value() * 16,g,0);
        else
            neopixelStrip.setAll(0,g,vol.value()*16);
        x = !x;  
    }
}
