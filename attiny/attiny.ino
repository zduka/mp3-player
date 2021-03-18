#include "Wire.h"

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
#define HEADPHONES 1
#define MIC 15

extern "C" void RTC_PIT_vect(void) __attribute__((signal));
extern "C" void ADC0_RESRDY_vect(void) __attribute__((signal));
extern "C" void ADC1_RESRDY_vect(void) __attribute__((signal));

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
        // enable the ADC inputs (audio, mic, voltage), disable the digital input buffers and pull-up resistors
        // NOTE this requires that the pins stay the same
        static_assert(AUDIO_ADC == 4, "Must be PB5"); // ADC0 input 8
        PORTB.PIN5CTRL &= ~PORT_ISC_gm;
        PORTB.PIN5CTRL |= PORT_ISC_INPUT_DISABLE_gc;
        PORTB.PIN5CTRL &= ~PORT_PULLUPEN_bm;
        static_assert(MIC == 15, "Must be PA2"); // ADC0 input 2
        PORTA.PIN2CTRL &= ~PORT_ISC_gm;
        PORTA.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;
        PORTA.PIN2CTRL &= ~PORT_PULLUPEN_bm;
        static_assert(VCC_SENSE == 0, "Must be PA4"); // ADC1 input 0
        PORTA.PIN4CTRL &= ~PORT_ISC_gm;
        PORTA.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;
        PORTA.PIN4CTRL &= ~PORT_PULLUPEN_bm;
        static_assert(HEADPHONES == 1, "Must be PA5"); // ADC1 input 1
        PORTA.PIN5CTRL &= ~PORT_ISC_gm;
        PORTA.PIN5CTRL |= PORT_ISC_INPUT_DISABLE_gc;
        PORTA.PIN5CTRL &= ~PORT_PULLUPEN_bm;
    }

    void setup() {
        // setup ADC0 and ADC1 settings
        // set reference to 1.1V
        VREF.CTRLA &= ~ VREF_ADC0REFSEL_gm;
        VREF.CTRLA |= VREF_ADC0REFSEL_1V1_gc;
        VREF.CTRLC &= ~ VREF_ADC1REFSEL_gm;
        VREF.CTRLC |= VREF_ADC1REFSEL_1V1_gc;
        // clkdiv by 4, internal voltage reference
        ADC0.CTRLC = ADC_PRESC_DIV4_gc | ADC_REFSEL_INTREF_gc;
        ADC1.CTRLC = ADC_PRESC_DIV4_gc | ADC_REFSEL_INTREF_gc;

        // get the voltage and determine if it's too low to proceed
        uint8_t voltage = getVoltage();
        powerOn();
        neopixels_.showBar(voltage, 50, Neopixel::Red());
        //        neopixels_.setAll(Neopixel::Green());
        neopixels_.sync();
        delay(1000);
        neopixels_.setAll(Neopixel::Black());
        neopixels_.sync();
    }
    
    void loop() {
        if (rtcTick()) {
            if (irq_.enabled && --irq_.timer == 0) {
                resetEsp();
            } else {
                // if the buttons are pressed, decrement their down ticks so that long presses can be determined 
                if (volBtnDownTicks_ > 0)
                    --volBtnDownTicks_;
                if (ctrlBtnDownTicks_ > 0)
                    --ctrlBtnDownTicks_;
                // update the lights
                lightsTick();
            }
        }
    }

private:

    void setMode(State::Mode mode) {
        state_.setMode(mode);
        switch (mode) {
            case State::Mode::MP3:
                enableAudioADC();
                digitalWrite(AUDIO_SRC, LOW);
                break;
            case State::Mode::Radio:
                enableAudioADC();
                digitalWrite(AUDIO_SRC, HIGH);
                break;
        }
    }

    void setIdle(bool value) {
        state_.setIdle(value);
        // TODO start idle timer to poweroff
    }
    
    void setVolume(uint8_t value) {
        if (value != state_.audioVolume()) {
            state_.setAudioVolume(value);
            neopixels_.showBar(state_.audioVolume(), 15, VOLUME_COLOR.withBrightness(brightness_));
            specialLights_ = SPECIAL_LIGHTS_TIMEOUT;
        }
    }

    void setControl(uint16_t value) {
        if (value != state_.control()) {
            state_.setControl(value);
            neopixels_.showPoint(state_.control(), control_.maxValue(), CONTROL_COLOR.withBrightness(brightness_));
            specialLights_ = SPECIAL_LIGHTS_TIMEOUT;
        }
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

    void powerOn() {
        digitalWrite(DCDC_PWR, HIGH);
        state_.state_ |= State::STATE_DCDC_POWER; 
        delay(50);
    }

    void powerOff() {
        digitalWrite(DCDC_PWR, LOW);
        state_.state_ &= ~State::STATE_DCDC_POWER;
    }

    /** Resets the ESP8266. 
     */
    void resetEsp() {
        clearIrq();
        neopixels_.setAll(Neopixel::Red());
        neopixels_.sync();
        delay(100);
        neopixels_.setAll(Neopixel::Black());
        neopixels_.sync();
        digitalWrite(DCDC_PWR, LOW);
        delay(500);
        digitalWrite(DCDC_PWR, HIGH);
        // TODO add wake signal
    }

    /** Sets the IRQ to ping the esp8266. 
     */
    bool setIrq() {
        if (!irq_.enabled) {
            irq_.enabled = true;
            irq_.timer = IRQ_MAX_DELAY;
            pinMode(AVR_IRQ,OUTPUT);
            digitalWrite(AVR_IRQ, LOW);
            return true;
        } else {
            return false;
        }
    }

    void clearIrq() {
        if (irq_.enabled) {
            irq_.enabled = false;
            state_.clearEvents();
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


    /** \name Lights

     */
    //@{
    NeopixelStrip<NEOPIXEL, 8> neopixels_;
    uint8_t specialLights_ = 0;
    Neopixel accentColor_ = Neopixel::White();
    /** Maximum brightness of the strip. 
     */
    uint8_t brightness_ = 32;

    void lightsTick() {
        if (state_.volumeButtonDown() || state_.controlButtonDown()) {
            uint8_t v = BUTTON_LONG_PRESS_TICKS - max(volBtnDownTicks_, ctrlBtnDownTicks_);
            if (v == BUTTON_LONG_PRESS_TICKS)
                neopixels_.setAll(BUTTONS_LONG_PRESS_COLOR.withBrightness(brightness_));
            else 
                neopixels_.showBar(v, BUTTON_LONG_PRESS_TICKS, BUTTONS_COLOR.withBrightness(brightness_));
        } else {
            if (specialLights_ == 0) {
                if (state_.audioLights())
                    updateAudioBar();
            } else if (--specialLights_ == 0) {
                neopixels_.setAll(Neopixel::Black());
            }
        }
        neopixels_.tick(max(brightness_ / 16, 1));
    }
    //@}

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
        if (volume_.poll()) {
            setVolume(volume_.value());
            setIrq();
        }
    }

    void controlChanged() {
        if (control_.poll()) {
            setControl(control_.value());
            setIrq();
        }
    }

    void volumeButtonChanged() {
        volumeButton_.poll();
        if (volumeButton_.pressed()) {
            volBtnDownTicks_ = BUTTON_LONG_PRESS_TICKS;
            state_.state_ |= State::STATE_VOL_BTN;
        } else {
            state_.state_ &= ~State::STATE_VOL_BTN;
            state_.events_ |= (volBtnDownTicks_ == 0) ? State::EVENT_VOL_LONG_PRESS : State::EVENT_VOL_PRESS;
            neopixels_.setAll(Neopixel::Black());
            setIrq();
        }
    }

    void controlButtonChanged() {
        controlButton_.poll();
        if (controlButton_.pressed()) {
            ctrlBtnDownTicks_ = BUTTON_LONG_PRESS_TICKS;
            state_.state_ |= State::STATE_CTRL_BTN;
        } else {
            state_.state_ &= ~State::STATE_CTRL_BTN;
            state_.events_ |= (ctrlBtnDownTicks_ == 0) ? State::EVENT_CTRL_LONG_PRESS : State::EVENT_CTRL_PRESS;
            neopixels_.setAll(Neopixel::Black());
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
            case Command::SetMode::Id: {
                Command::SetMode * cmd = pointer_cast<Command::SetMode*>(& cmdBuffer_[1]);
                setMode(cmd->mode);
                break;
            }
            case Command::SetIdle::Id: {
                Command::SetIdle * cmd = pointer_cast<Command::SetIdle*>(& cmdBuffer_[1]);
                setIdle(cmd->idle);
                break;
            }
            case Command::SetVolume::Id: {
                Command::SetVolume * cmd = pointer_cast<Command::SetVolume*>(& cmdBuffer_[1]);
                volume_.setValue(cmd->value);
                setVolume(cmd->value);
                break;
            }
            case Command::SetControl::Id: {
                Command::SetControl * cmd = pointer_cast<Command::SetControl*>(& cmdBuffer_[1]);
                control_.setMaxValue(cmd->maxValue);
                control_.setValue(cmd->value);
                setControl(cmd->value);
                break;
            }
            case Command::SetRadioState::Id: {
                Command::SetRadioState * cmd = pointer_cast<Command::SetRadioState*>(& cmdBuffer_[1]);
                state_.setRadioStation(cmd->station);
                state_.setRadioFrequency(cmd->frequency);
                state_.setRadioManualTuning(cmd->manualTuning);
                break;
            }
                
            case Command::SetAudioLights::Id: {
                Command::SetAudioLights * cmd = pointer_cast<Command::SetAudioLights*>(& cmdBuffer_[1]);
                state_.setAudioLights(cmd->on);
                neopixels_.setAll(Neopixel::Black());
                break;
            }
            case Command::SetBrightness::Id: {
                Command::SetBrightness * cmd = pointer_cast<Command::SetBrightness*>(& cmdBuffer_[1]);
                brightness_ = cmd->brightness;
                break;
            }
            case Command::SetAccentColor::Id: {
                Command::SetAccentColor * cmd = pointer_cast<Command::SetAccentColor*>(& cmdBuffer_[1]);
                accentColor_.r = cmd->red;
                accentColor_.g = cmd->green;
                accentColor_.b = cmd->blue;
                break;
            }
            case Command::SpecialLights::Id: {
                Command::SpecialLights * cmd = pointer_cast<Command::SpecialLights*>(& cmdBuffer_[1]);
                Neopixel color{cmd->red, cmd->green, cmd->blue};
                switch (cmd->mode) {
                    case Command::SpecialLights::POINT:
                        neopixels_.showPoint(cmd->value, cmd->maxValue, color);
                        break;
                    case Command::SpecialLights::BAR:
                        neopixels_.showBar(cmd->value, cmd->maxValue, color);
                        break;
                    case Command::SpecialLights::CENTERED_BAR:
                        neopixels_.showCenteredBar(cmd->value, cmd->maxValue, color);
                        break;
                }
                specialLights_ = cmd->duration;
                break;
            }
            case Command::Lights::Id: {
                Command::Lights * cmd = pointer_cast<Command::Lights*>(& cmdBuffer_[1]);
                for (uint8_t i = 0; i < 8; ++i) {
                    Neopixel & p = neopixels_[i];
                    p.r = cmd->colors[i * 3];
                    p.g = cmd->colors[i * 3 + 1];
                    p.b = cmd->colors[i * 3 + 2];
                }
                specialLights_ = cmd->duration;
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








    /** RTC ticks counter, there are 32 ticks in one second. 
     */
    mutable volatile struct {
        unsigned tick : 1;
        unsigned tickSecond : 1;
        unsigned counter : 5; // 0..31
    } rtcTick_;

    mutable volatile struct {
        unsigned enabled : 1;
        unsigned timer : 7;
    } irq_;


    /** \name ADC1 - Voltage, temperature and headphones
     */
    //@{

    uint8_t getVoltage() {
        ADC1.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_8BIT_gc;
        ADC1.MUXPOS = ADC_MUXPOS_AIN0_gc;
        ADC1.COMMAND = ADC_STCONV_bm;
        // wait for the conversion to be complete
        while (ADC1.INTFLAGS & ADC_RESRDY_bm == 0)
            continue;
        // convert the result to voltage and store it in the state
        setVoltageFromADC(ADC1.RES);
        return state_.voltage();
    }

    void setVoltageFromADC(uint8_t adc) {
        state_.setVoltage(static_cast<uint8_t>(11.0 * adc / 255 * (VCC_DIVIDER_RES1 + VCC_DIVIDER_RES2) / VCC_DIVIDER_RES2));
    }

    //@}

    

    /** \name Audio output and mic readouts. 
     */
    //@{


    void enableAudioADC() {
        audioMin_ = 255;
        audioMax_ = 0;
        audioSamples_ = 0;
        // enable and use 8bit resolution, freerun mode
        ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_8BIT_gc | ADC_FREERUN_bm;
        // select ADC channel to AUDIO_ADC pin
        ADC0.MUXPOS  = ADC_MUXPOS_AIN8_gc;
        // enable the interrupt
        ADC0.INTCTRL |= ADC_RESRDY_bm;
        // and start the conversion
        ADC0.COMMAND = ADC_STCONV_bm;        
    }
    
    void addAudioReading(uint8_t value) {
        ++audioSamples_;
        if (value < audioMin_)
            audioMin_ = value;
        if (value > audioMax_)
            audioMax_ = value;
    }

    void updateAudioBar() {
        uint8_t v = audioMax_ - audioMin_;
        v = v < 32 ? 0 : v - 32;
        neopixels_.showCenteredBar(v, 128, AUDIO_COLOR.withBrightness(brightness_));
        //neopixels_.sync();
        audioMin_ = 255;
        audioMax_ = 0;
        audioSamples_ = 0;
    }
    
    uint8_t audioMin_;
    uint8_t audioMax_;
    uint16_t audioSamples_;
    //@}
    

    static void I2CSendEventTrampoline();
    static void I2CReceiveEventTrampoline(int numBytes);

    static void VolumeChangedTrampoline(); 
    static void VolumeButtonChangedTrampoline();
    static void ControlChangedTrampoline();
    static void ControlButtonChangedTrampoline();

    friend void ::RTC_PIT_vect();
    friend void ::ADC0_RESRDY_vect();
    friend void ::ADC1_RESRDY_vect();

}; // Player


AVRPlayer player; // the player singleton

/** Real-time clock tick interrupt. 
    
    When running, happens every 1/32th of a second. When sleeping, should happen every second only. 
 */
ISR(RTC_PIT_vect) {
   RTC.PITINTFLAGS = RTC_PI_bm;
   player.doRtcTick();
}

ISR(ADC0_RESRDY_vect) {
    player.addAudioReading(ADC0.RES); 
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
    player.setup();
}



/** During the loop, the following things must be checked:

    - determine the input voltage by reading the VCC_SENSE (low)
    
 */
void loop() {
    player.loop();
}
