#include "Wire.h"

//#include <tinyNeoPixel_Static.h>

#include "state.h"
#include "inputs.h"
#include "neopixel.h"

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

#define BUTTON_LONG_PRESS_TICKS 64


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
#define AUDIO_ADC 4
#define VCC_SENSE 0
#define MIC 15

extern "C" void RTC_PIT_vect(void) __attribute__((signal));

/** The MP3 Player Device Controller

    # RTC

    The internal RTC in the attiny is used to keep a semi-precise track of time. When the chip is on, a tick is generated every 1/32th of second so that it can be used to drive neopixels, etc. When the device sleeps, a one second tick is used to save battery and only the time is updated and alarm checked. 
 */
class AVRPlayer {
public:

    AVRPlayer() {
        // enable control interrupts
        volume_.setInterrupt(VolumeChangedTrampoline);
        control_.setInterrupt(ControlChangedTrampoline);
        volumeButton_.setInterrupt(VolumeButtonChangedTrampoline);
        controlButton_.setInterrupt(ControlButtonChangedTrampoline);
        
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
        Wire.onRequest(I2CSendEventTrampoline);
        Wire.onReceive(I2CReceiveEventTrampoline);
        // set audio source to esp8266
        pinMode(AUDIO_SRC, OUTPUT);
        digitalWrite(AUDIO_SRC, LOW);
        // enable the ADC inputs (audio, mic, voltage)
        pinMode(AUDIO_ADC, INPUT);
        pinMode(MIC, INPUT);
        pinMode(VCC_SENSE, INPUT);

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
            neopixels_.showBar(state_.volume(), 15, Neopixel::Blue());
            setIrq();
        }
    }

    void setControl(uint16_t value) {
        if (value != state_.control()) {
            state_.setControl(value);
            neopixels_.showPoint(state_.control(), control_.maxValue(), Neopixel::Green());
            setIrq();
        }
    }

    void enterMP3Mode(uint16_t controlValue, uint16_t controlMax) {
        control_.setMaxValue(controlMax);
        control_.setValue(controlValue);
        digitalWrite(AUDIO_SRC, LOW);
        state_.setAudioSrc(State::AudioSrc::ESP);
        state_.setMode(State::Mode::MP3);
        state_.setControl(control_.value());
        // make sure we have IRQ set
        setIrq();
    }

    /** Enters the radio mode. 
     */
    void enterRadioMode(uint16_t controlValue, uint16_t controlMax) {
        control_.setMaxValue(controlMax);
        control_.setValue(controlValue);
        digitalWrite(AUDIO_SRC, HIGH);
        state_.setAudioSrc(State::AudioSrc::Radio);
        state_.setMode(State::Mode::Radio);
        state_.setControl(control_.value());
        // make sure we have IRQ set
        setIrq();
    }

    void setup() {
        neopixels_.setAll(Neopixel::Green());
        neopixels_.update();
        delay(100);
        neopixels_.setAll(Neopixel::Black());
        neopixels_.update();
    }
    
    void loop() {
        if (rtcTick()) {
            neopixels_.tick();
            //neopixels_.update();
        }
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

    NeopixelStrip<NEOPIXEL, 8> neopixels_;

    /** \name Controls
     */
    //@{

    /** The control (top) and volume (down) knobs (rotary encoders and buttons). 
     */
    RotaryEncoder control_{CTRL_A, CTRL_B, 256};
    RotaryEncoder volume_{VOL_A, VOL_B, 16};
    Button controlButton_{CTRL_BTN};
    Button volumeButton_{VOL_BTN};
    uint8_t volBtnDownTicks_ = 0;
    uint8_t ctrlBtnDownTicks_ = 0;

    void volumeChanged() {
        volume_.poll();
        setVolume(volume_.value());
    }

    void controlChanged() {
        control_.poll();
        setControl(control_.value());
    }

    void volumeButtonChanged() {
        volBtnDownTicks_ = BUTTON_LONG_PRESS_TICKS;
        volumeButton_.poll();
        if (volumeButton_.pressed()) {
            state_.state_ |= State::STATE_VOL_BTN;
        } else {
            state_.state_ &= ~State::STATE_VOL_BTN;
            state_.events_ |= (volBtnDownTicks_ == 0) ? State::EVENT_VOL_LONG_PRESS : State::EVENT_VOL_PRESS;
            setIrq();
        }
    }

    void controlButtonChanged() {
        ctrlBtnDownTicks_ = BUTTON_LONG_PRESS_TICKS;
        controlButton_.poll();
        if (controlButton_.pressed()) {
            state_.state_ |= State::STATE_CTRL_BTN;
        } else {
            state_.state_ &= ~State::STATE_CTRL_BTN;
            state_.events_ |= (ctrlBtnDownTicks_ == 0) ? State::EVENT_CTRL_LONG_PRESS : State::EVENT_CTRL_PRESS;
            setIrq();
        }
    }
    //@}

    /** \name I2C 
     */
    //@{

    uint8_t cmdBuffer_[32];
    
    void i2cSendEvent() {
        // casting away the volatile-ness is ok here 
        Wire.write(pointer_cast<uint8_t*>(const_cast<State*>(& state_)), sizeof (State));
        // clear the irq and events once we have given them to master
        clearIrq();
        state_.events_ = 0;
    }

    void i2cReceiveEvent(int numBytes) {
        Wire.readBytes(pointer_cast<uint8_t*>(& cmdBuffer_), numBytes);
        switch (cmdBuffer_[0]) {
            case Command::EnterMP3Mode::Id: {
                Command::EnterRadioMode * cmd = pointer_cast<Command::EnterRadioMode*>(& cmdBuffer_[1]);
                enterMP3Mode(cmd->controlValue, cmd->controlMaxValue);
                break;
            }
            case Command::EnterRadioMode::Id: {
                Command::EnterRadioMode * cmd = pointer_cast<Command::EnterRadioMode*>(& cmdBuffer_[1]);
                enterRadioMode(cmd->controlValue, cmd->controlMaxValue);
                break;
            }
            case Command::SetVolume::Id: {
                Command::SetVolume * cmd = pointer_cast<Command::SetVolume*>(& cmdBuffer_[1]);
                volume_.setValue(cmd->value);
                state_.setVolume(volume_.value(), /* recordChange */ false);
                break;
            }
            case Command::SetControl::Id: {
                Command::SetControl * cmd = pointer_cast<Command::SetControl*>(& cmdBuffer_[1]);
                control_.setMaxValue(cmd->maxValue);
                control_.setValue(cmd->value);
                state_.setControl(control_.value(), /* recordChange */ false);
                break;
            }
            default:
                break;
        }
    }
    //@}

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

    
    static void I2CSendEventTrampoline();
    static void I2CReceiveEventTrampoline(int numBytes);

    static void VolumeChangedTrampoline(); 
    static void VolumeButtonChangedTrampoline();
    static void ControlChangedTrampoline();
    static void ControlButtonChangedTrampoline();

    friend void ::RTC_PIT_vect();

}; // Player


AVRPlayer player; // the player singleton

/** Real-time clock tick interrupt. 
    
    When running, happens every 1/32th of a second. When sleeping, should happen every second only. 
 */
ISR(RTC_PIT_vect) {
   RTC.PITINTFLAGS = RTC_PI_bm;
   player.doRtcTick();
}


void AVRPlayer::I2CSendEventTrampoline() {
    player.i2cSendEvent();
}

void AVRPlayer::I2CReceiveEventTrampoline(int numBytes) {
    player.i2cReceiveEvent(numBytes);
}

void AVRPlayer::VolumeChangedTrampoline() {
    player.volumeChanged();
}

void AVRPlayer::VolumeButtonChangedTrampoline() {
    player.volumeButtonChanged();
}

void AVRPlayer::ControlChangedTrampoline() {
    player.controlChanged();
}

void AVRPlayer::ControlButtonChangedTrampoline() {
    player.controlButtonChanged();
}

void setup() {
    /*
    Serial.swap();
    Serial.begin(9600);
    Serial.println("Hello world");
    */

    

    player.requestPowerOn();
    // quick blink at startup
    player.setup();

}



/** During the loop, the following things must be checked:

    - determine the input voltage by reading the VCC_SENSE (low)
    
 */
void loop() {
    player.loop();
}
