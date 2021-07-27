#if (defined ARCH_ATTINY)

#if (F_CPU!=8000000UL)
#error "Only 8Mhz clock is supported"
#endif


#include <avr/sleep.h>
#include <util/delay.h>
#include <Wire.h>

#include "state.h"
#include "messages.h"
#include "attiny/inputs.h"
#include "attiny/neopixel.h"

/** Chip Pinout

               -- VDD             GND --
         VOL_B -- (00) PA4   PA3 (16) -- CTRL_A
       VOL_BTN -- (01) PA5   PA2 (15) -- CTRL_BTN
         VOL_A -- (02) PA6   PA1 (14) -- HEADPHONES
      DCDC_PWR -- (03) PA7   PA0 (17) -- UPDI
               -- (04) PB5   PC3 (13) -- CTRL_B
      CHARGING -- (05) PB4   PC2 (12) -- AUDIO_ADC
      NEOPIXEL -- (06) PB3   PC1 (11) -- AUDIO_SRC ????? was it here? 
       AVR_IRQ -- (07) PB2   PC0 (10) -- MIC
           SDA -- (08) PB1   PB0 (09) -- SCL

TODO change AVR_IRQ to PB5 and set PB2 for TX for serial, this would make debugging less painful
but we actually need audio_src
 */

#define DCDC_PWR 3
#define NEOPIXEL 6
#define CTRL_A 16
#define CTRL_B 13
#define CTRL_BTN 15
#define VOL_A 2
#define VOL_B 0
#define VOL_BTN 1
#define AVR_IRQ 7
#define AUDIO_SRC 11
#define AUDIO_ADC 12
#define HEADPHONES 14
#define MIC 10

extern "C" void RTC_PIT_vect(void) __attribute__((signal));
extern "C" void ADC0_RESRDY_vect(void) __attribute__((signal));
extern "C" void ADC1_RESRDY_vect(void) __attribute__((signal));
//extern "C" void TCB0_INT_vect(void) __attribute__((signal));
/** ATTiny part of the player.     
    
 */
class Player {
public:
    Player() = delete;

    /** Initializes the chip. 
     */
    static void Initialize() {
        // disable power to other systems
        pinMode(DCDC_PWR, INPUT);
        // enable the AVR_IRQ as input so that we can observe if ESP wants something
        pinMode(AVR_IRQ, INPUT);
        // headphones are input pin as well
        pinMode(HEADPHONES, INPUT);
        // audio source is output, set it low by default
        pinMode(AUDIO_SRC, OUTPUT);
        digitalWrite(AUDIO_SRC, LOW);
        // disable the peripheral clock divider
        //CPU_CCP = CCP_IOREG_gc;
        //CLKCTRL.MCLKCTRLB &= ~ CLKCTRL_PEN_bm; 
        // configure the real time clock
        RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; // select internal oscillator
        RTC.PITINTCTRL |= RTC_PI_bm; // enable the interrupt
        // configure the timer we use for 8kHz audio sampling
        TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc;
        TCB0.CTRLB = TCB_CNTMODE_INT_gc;
        TCB0.CCMP = 1000; // for 8kHz
        // set sleep to full power down and enable sleep feature
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();     
        // setup the ADC pins (microphone input & audio out)
        static_assert(AUDIO_ADC == 12, "Must be PC2"); // ADC1 input 8
        PORTC.PIN2CTRL &= ~PORT_ISC_gm;
        PORTC.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;
        PORTC.PIN2CTRL &= ~PORT_PULLUPEN_bm;
        static_assert(MIC == 10, "Must be PC0"); // ADC1 input 6
        PORTC.PIN0CTRL &= ~PORT_ISC_gm;
        PORTC.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
        PORTC.PIN0CTRL &= ~PORT_PULLUPEN_bm;
        // setup ADC voltage references to 1.1V (internal)
        VREF.CTRLA &= ~ VREF_ADC0REFSEL_gm;
        VREF.CTRLA |= VREF_ADC0REFSEL_1V1_gc;
        VREF.CTRLC &= ~ VREF_ADC1REFSEL_gm;
        VREF.CTRLC |= VREF_ADC1REFSEL_1V1_gc;
        // configure the event system - ADC1 to sample when triggered by TCB0
        EVSYS.SYNCCH0 = EVSYS_SYNCCH0_TCB0_gc;
        EVSYS.ASYNCUSER12 = EVSYS_ASYNCUSER12_SYNCCH0_gc;
        // set ADC1 settings (mic & audio) - clkdiv by 4, internal voltage reference
        ADC1.CTRLC = ADC_PRESC_DIV4_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm; // 2mhz
        ADC1.EVCTRL = ADC_STARTEI_bm;
        // set ADC0 settings (vcc, temperature)
        // delay 32us and sampctrl of 32 us for the temperature sensor, do averaging over 64 values
        ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
        ADC0.CTRLB = ADC_SAMPNUM_ACC64_gc;
        ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
        ADC0.CTRLD = ADC_INITDLY_DLY32_gc;
        ADC0.SAMPCTRL = 31;
        // initialize I2C slave. 
        Wire.begin(AVR_I2C_ADDRESS);
        Wire.onRequest(I2CRequest);
        Wire.onReceive(I2CReceive);
        // enable control interrupts for buttons
        ControlBtn_.setInterrupt(ControlButtonChanged);
        VolumeBtn_.setInterrupt(VolumeButtonChanged);
        // allow voltages to stabilize when powered up
        delay(200);
        // from now on proceed identically to a wakeup
        Wakeup();
        StartAudioADC(AudioADCSource::Mic);
    }

    static void Loop() {
        if (Status_.sleep)
            Sleep();
            
        if (Status_.tick) {
            Status_.tick = false;
            LightsTick();
        }
        if (Status_.secondTick) {
            Status_.secondTick = false;
            if (State_.voltage() <= BATTERY_CRITICAL) {
                CriticalBattery();
                return;
            }
        }
    }

private:

    static void Tick() {
        // if sleeping & button(s) are pressed, we have 1/32th of a second tick and must determine if wake up
        // if not sleeping, the long press counters must be increased and second tick determined
        if (! Status_.sleep || State_.controlDown() || State_.volumeDown()) {
            bool wakeup = false;
            if (State_.controlDown() && ControlBtnCounter_ > 0 && --ControlBtnCounter_ == 0) 
                wakeup = true;
            if (State_.volumeDown() && VolumeBtnCounter_ > 0 && --VolumeBtnCounter_ ==0) 
                wakeup = true;
            // if sleeping, check if we should wake up, otherwise initiate the tick in main loop
            if (Status_.sleep) {
                if (wakeup) {
                    State_.setControlDown(false);
                    State_.setVolumeDown(false);
                    State_.clearEvents();
                    Status_.sleep = false;
                }
            } else {
                Status_.tick = true;
            }
            // if this is the 32th tick, continue to second tick, otherwise return now            
            if (Player::Status_.ticksCounter-- != 0)
                return;
        }
        // we are left with 1 second tick either because sleeping, or because fast tick overflow
        Status_.secondTick = true;
        Time_.secondTick();
        // TODO alarm & stuff
    }

    /** Initialized when the chip wakes up into normal operation. 
     */
    static void Wakeup() {
        // enable the RTC interrupt to 1/64th of a second while awake
        while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
        RTC.PITCTRLA = RTC_PERIOD_CYC512_gc + RTC_PITEN_bm;
        // start ADC0 to measure the VCC and wait for the first measurement to be done
        ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
        ADC0.INTCTRL |= ADC_RESRDY_bm;
        ADC0.COMMAND = ADC_STCONV_bm;
        // wait for the voltage measurement to finish
        while (ADC0.MUXPOS == ADC_MUXPOS_INTREF_gc) {
        }
        // if the voltage is below low battery threshold
        if (State_.voltage() <= BATTERY_CRITICAL) {
            CriticalBattery();
            return;
        }
        // enable interrupts for rotary encoders
        Control_.setInterrupt(ControlValueChanged);
        Volume_.setInterrupt(VolumeValueChanged);
        // turn on ESP and neopixels
        // TODO have this in a method so that it can be called from reset function as well? 
        pinMode(DCDC_PWR, OUTPUT);
        digitalWrite(DCDC_PWR, LOW);
        delay(50);
        // start the wakeup progress bar
        SpecialLights_.showBar(1, 5, Color::White().withBrightness(MaxBrightness_));
        LightsCounter_ = 64;
    }

    static void Sleep() {
        do {
            // enable RTC interrupt every second so that the time can be kept
            while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
            RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc + RTC_PITEN_bm;
            // disable rotary encoder interrupts so that they do not wake us from sleep
            Control_.clearInterrupt();
            Volume_.clearInterrupt();
            // turn off neopixels and esp
            pinMode(DCDC_PWR, INPUT);
            // enter the sleep mode. Upon wakeup, go to sleep immediately for as long as the sleep mode is on (we wake up every second to increment the clock and when buttons are pressed as well)
            Status_.sleep = true;
            while (Status_.sleep) {
                sleep_cpu();
            }
            // clear the button events that led to the wakeup
            State_.clearButtonEvents();
            // if we are not longer sleeping, wakeup
            Wakeup();
        } while (Status_.sleep == true);
    }

    /** Flashes the strip three times with red as a critical battery indicator. 
     */
    static void CriticalBattery() {
        Wire.end();
        pinMode(SDA, OUTPUT);
        pinMode(SCL, OUTPUT);
        digitalWrite(SDA, LOW);
        digitalWrite(SCL, LOW);
        pinMode(DCDC_PWR, OUTPUT);
        digitalWrite(DCDC_PWR, LOW);
        delay(50);
        Neopixels_.fill(Color::Red().withBrightness(DEFAULT_NOTIFICATION_BRIGHTNESS));
        Neopixels_.update();
        delay(50);
        Neopixels_.fill(Color::Black());
        Neopixels_.update();
        delay(100);
        Neopixels_.fill(Color::Red().withBrightness(DEFAULT_NOTIFICATION_BRIGHTNESS));
        Neopixels_.update();
        delay(50);
        Neopixels_.fill(Color::Black());
        Neopixels_.update();
        delay(100);
        Neopixels_.fill(Color::Red().withBrightness(DEFAULT_NOTIFICATION_BRIGHTNESS));
        Neopixels_.update();
        delay(50);
        Neopixels_.fill(Color::Black());
        Neopixels_.update();
        Wire.begin(AVR_I2C_ADDRESS);
        Status_.sleep = true;
    }


    friend void ::RTC_PIT_vect();

    friend void ::ADC0_RESRDY_vect();

    inline volatile static struct {
        bool sleep : 1;
        bool tick : 1;
        bool secondTick : 1;
        uint8_t ticksCounter : 6; // 0..63
    } Status_;

    inline static State State_;

    inline static volatile DateTime Time_;

/** \name Comms
 */
//@{
private:

    static void SetIrq() {
        if (!Irq_.enabled) {
            Irq_.enabled = true;
            Irq_.timer = IRQ_MAX_DELAY;
            pinMode(AVR_IRQ, OUTPUT);
            digitalWrite(AVR_IRQ, LOW);
        }
    }

    static void ClearIrq() {
        if (Irq_.enabled) {
            Irq_.enabled = false;
            pinMode(AVR_IRQ, INPUT);
        }
    }

    static void ResetESP() {
        // this is necessary so that ESP boots into normal mode
        ClearIrq();
    }


    /** Triggered when ESP requests data to be sent. 
     
        By default, the state bytes are sent.
     */
    static void I2CRequest() {
        //digitalWrite(AUDIO_SRC, HIGH);
        switch (I2CSource_) {
            case I2CSource::State:
                Wire.write(pointer_cast<uint8_t*>(& State_), sizeof (State));
                State_.clearEvents();
                ClearIrq();
        }
        //digitalWrite(AUDIO_SRC, LOW);
    }

    static void I2CReceive(int numBytes) {
        // TODO check that numBytes <=32
        Wire.readBytes(pointer_cast<uint8_t*>(& Buffer_), numBytes);
        switch (Buffer_[0]) {
            case msg::PowerOff::Id: {
                // set sleep to true so that the sleep can be handled in main loop
                //Status_.sleep = true;
                break;
            }
            case msg::SetMode::Id: {
                auto msg = msg::At<msg::SetMode>(Buffer_);
                msg.applyTo(State_);
                // sync the volume and control states
                Control_.setValues(State_.control(), State_.maxControl());
                Volume_.setValues(State_.volume(), State_.maxVolume());
                // sync the effect color for the night mode
                EffectColor_ = State_.nightLightColor(AccentColor_, MaxBrightness_);
                break;
            }
            case msg::SetWiFiStatus::Id: {
                auto msg = msg::At<msg::SetWiFiStatus>(Buffer_);
                msg.applyTo(State_);
                break;
            }
            case msg::SetAudioSource::Id: {
                auto msg = msg::At<msg::SetAudioSource>(Buffer_);
                msg.applyTo(State_);
                break;
            }
            case msg::SetMP3Settings::Id: {
                auto msg = msg::At<msg::SetMP3Settings>(Buffer_);
                msg.applyTo(State_);
                break;
            }
            case msg::SetRadioSettings::Id: {
                auto msg = msg::At<msg::SetRadioSettings>(Buffer_);
                msg.applyTo(State_);
                break;
            }
            case msg::SetNightLightSettings::Id: {
                auto msg = msg::At<msg::SetNightLightSettings>(Buffer_);
                msg.applyTo(State_);
                EffectColor_ = State_.nightLightColor(AccentColor_, MaxBrightness_);
                break;
            }
            case msg::SetAccentColor::Id: {
                auto msg = msg::At<msg::SetAccentColor>(Buffer_);
                AccentColor_ = msg.color;
                break;
            }
            case msg::LightsBar::Id: {
                auto msg = msg::At<msg::LightsBar>(Buffer_);
                SpecialLights_.showBar(msg.value, msg.max, msg.color);
                LightsCounter_ = msg.timeout;
                break;
            }
            case msg::LightsCenteredBar::Id: {
                auto msg = msg::At<msg::LightsCenteredBar>(Buffer_);
                SpecialLights_.showCenteredBar(msg.value, msg.max, msg.color);
                LightsCounter_ = msg.timeout;
                break;
            }
            case msg::LightsPoint::Id: {
                auto msg = msg::At<msg::LightsPoint>(Buffer_);
                SpecialLights_.showPoint(msg.value, msg.max, msg.color);
                LightsCounter_ = msg.timeout;
                break;
            }
            case msg::LightsColors::Id: {
                auto msg = msg::At<msg::LightsColors>(Buffer_);
                for (int i = 0; i < 8; ++i)
                    SpecialLights_[i] = msg.colors[i];
                LightsCounter_ = msg.timeout;
                break;
            }
        }
    }

    enum class I2CSource : uint8_t {
        State
    };

    static_assert(IRQ_MAX_DELAY < 128);
    inline volatile static struct {
        bool enabled : 1;
        uint8_t timer : 7; // 0..127
    } Irq_;

    inline static I2CSource I2CSource_ = I2CSource::State;

    inline static uint8_t Buffer_[32];


//@}



/** \name Knobs & Buttons
 
    
 */
//@{

    inline static RotaryEncoder Control_{CTRL_A, CTRL_B, 256};
    inline static RotaryEncoder Volume_{VOL_A, VOL_B, 16};
    inline static Button ControlBtn_{CTRL_BTN};
    inline static Button VolumeBtn_{VOL_BTN};
    inline static uint8_t ControlBtnCounter_ = 0;
    inline static uint8_t VolumeBtnCounter_ = 0;

    static void ControlValueChanged() {
        Control_.poll();
        State_.setControl(Control_.value());
        SetIrq();
    }   

    static void VolumeValueChanged() {
        Volume_.poll(); 
        State_.setVolume(Volume_.value());
        SetIrq();
    } 

    static void ControlButtonChanged() {
        ControlBtn_.poll();
        if (ControlBtn_.pressed()) {
            ControlBtnCounter_ = BUTTON_LONG_PRESS_TICKS;
            State_.setControlDown(true);
            // if we are sleeping, increase RTC period to 1/32th of a second so that we can detect the long tick
            if (Status_.sleep) {
                while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
                RTC.PITCTRLA = RTC_PERIOD_CYC512_gc + RTC_PITEN_bm;     
            }
        } else if (State_.controlDown()) {
            State_.setControlDown(false);
            if (ControlBtnCounter_ == 0) {
                State_.setControlLongPress();
            } else {
                ControlBtnCounter_ = 0;
                State_.setControlPress();
            }
        }
        // if we are not sleeping, set the IRQ to notify ESP. If we have just woken up, then this is irrelevant and will be cleared when waking ESP up
        if (! Status_.sleep)
            SetIrq();
    }

    static void VolumeButtonChanged() {
        VolumeBtn_.poll();
        if (VolumeBtn_.pressed()) {
            VolumeBtnCounter_ = BUTTON_LONG_PRESS_TICKS;
            State_.setVolumeDown(true);
            // if we are sleeping, increase RTC period to 1/32th of a second so that we can detect the long tick
            if (Status_.sleep) {
                while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
                RTC.PITCTRLA = RTC_PERIOD_CYC512_gc + RTC_PITEN_bm;     
            }
        } else {
            State_.setVolumeDown(false);
            if (VolumeBtnCounter_ == 0) {
                State_.setVolumeLongPress();
                // wake up from sleep if sleeping
                Status_.sleep = false;
            } else {
                VolumeBtnCounter_ = 0;
                State_.setVolumePress();
            }
        }
        // if we are not sleeping, set the IRQ to notify ESP. If we have just woken up, then this is irrelevant and will be cleared when waking ESP up
        if (! Status_.sleep)
            SetIrq();
    }


//@}


/** \name Lights
 */
//@{

    /** Determines what to show with the lights. 
     
        The following things are displayed on the strip:

     */ 
    static void LightsTick() {
        // always start fresh
        uint8_t step = max(MaxBrightness_ / 32, 1);
        // when a button is down then the strip shows the long press progress first and foremost
        if (State_.volumeDown() || State_.controlDown()) {
            uint8_t v = BUTTON_LONG_PRESS_TICKS - max(VolumeBtnCounter_, ControlBtnCounter_);
            if (v == BUTTON_LONG_PRESS_TICKS)
                Lights_.fill(BUTTONS_LONG_PRESS_COLOR.withBrightness(MaxBrightness_));
            else
                Lights_.showBar(v, BUTTON_LONG_PRESS_TICKS, BUTTONS_LONG_PRESS_COLOR.withBrightness(MaxBrightness_));
        // otherwise, if LightsCounter_ is non-zero, it means that the special lights are to be shown, in which case we simply perform the tick against the special lights buffer instead.   
        } else if (LightsCounter_ > 0) {
            Lights_.moveTowards(SpecialLights_, step);
            --LightsCounter_ == 0;
        // otherwise, show network connecting bar, if currently connecting to a network
        } else if (State_.wifiStatus() == WiFiStatus::Connecting) {
            Lights_.showCenteredBar(Status_.ticksCounter, 63, Color::Blue().withBrightness(MaxBrightness_));
        // if in night light mode, show the selected effect
        } else if (State_.mode() == Mode::NightLight) {
            step = 255; // pixels will be synced, not moved towards
            switch (State_.nightLightEffect()) {
                case NightLightEffect::Color:
                    Lights_.fill(EffectColor_);
                    break;
                case NightLightEffect::Breathe:
                    Lights_.fill(EffectColor_.withBrightness(EffectCounter_ >> 8));
                    break;
                case NightLightEffect::BreatheBar:
                    Lights_.showCenteredBar(EffectCounter_, 0xffff, EffectColor_);
                    break;
                case NightLightEffect::KnightRider:
                    Lights_.showPoint(EffectCounter_, 0xffff, EffectColor_);
                    break;
                case NightLightEffect::Running:
                    break;
            }
            uint16_t speed = 16 * 32;
            if (EffectHelper_ & 0x8000) {
                EffectCounter_ += speed;
                if (EffectCounter_ < speed) {
                    EffectCounter_ = 0xffff;
                    EffectHelper_ &= ~0x8000;
                }
            } else {
                EffectCounter_ -= speed;
                if (EffectCounter_ > 0xffff - speed) {
                    EffectCounter_ = 0;
                    EffectHelper_ |= 0x8000;
                }
            }
            // if we are in rainbow mode, update the effect color hue
            if (State_.nightLightHue() == State::NIGHTLIGHT_RAINBOW_HUE) {
                uint16_t hue = EffectHelper_ & 0xfff;
                hue += 16;
                if (hue > 0xfff)
                    hue = 0;
                EffectHelper_ = (EffectHelper_ & 0xf000) | hue;
                EffectColor_ = Color::HSV(hue << 4, 255, MaxBrightness_);
            }
        // display audio lights now
        // TODO remove the true flag and actually work this based on mode & audio lights state
        } else if (State_.audioLights() || true) {
            uint8_t v = AudioMax_ - AudioMin_;
            AudioMin_ = 255;
            AudioMax_ = 0;
            if (v > AudioLightsMax_)
                AudioLightsMax_ = v;
            Lights_.showCenteredBar(v, AudioLightsMax_, AccentColor_.withBrightness(MaxBrightness_));
            if (AudioLightsMax_ > 0 && Status_.ticksCounter % 4 == 0)
                --AudioLightsMax_;
            //step = 255;
        // otherwise the lights are off
        } else {
            Lights_.fill(Color::Black());
        }
        // update the actual neopixels buffer
        Neopixels_.moveTowardsReversed(Lights_, step);
        // and finally, draw over the bar notification lights - since we do this every tick, calculate the blinking brightness transition first
        uint16_t notificationBrightness = DEFAULT_NOTIFICATION_BRIGHTNESS;
        if (Status_.ticksCounter < 16)
            notificationBrightness = notificationBrightness * Status_.ticksCounter / 16;
        else if (Status_.ticksCounter > 48)
            notificationBrightness = notificationBrightness * (63 - Status_.ticksCounter) / 16;
        // after done, the bar as such, graft on it any notification LEDs with have
        if (Time_.second() % 2) {
            if (State_.wifiStatus() == WiFiStatus::Connected || State_.wifiStatus() == WiFiStatus::SoftAP)
                Neopixels_[0].add(Color::Blue().withBrightness(notificationBrightness));
        // the low battery warning blinks out of sync with the other notifications
        } else {
            if (State_.voltage() <= BATTERY_LOW)
                Neopixels_[7].add(Color::Red().withBrightness(notificationBrightness));
        }
        // and finally, update the neopixels
        Neopixels_.update();
    }


    inline static NeopixelStrip<NEOPIXEL, 8> Neopixels_;
    inline static ColorStrip<8> Lights_;
    inline static ColorStrip<8> SpecialLights_;
    inline static volatile uint8_t MaxBrightness_ = DEFAULT_BRIGHTNESS;
    inline static volatile uint16_t EffectCounter_ = 0;
    inline static volatile uint16_t EffectHelper_ = 0;
    inline static Color AccentColor_ = DEFAULT_ACCENT_COLOR;
    inline static Color EffectColor_;
    inline static volatile uint8_t LightsCounter_ = 0;

//@}


/** \name Audio & Microphone Listening
 */
//@{
private:

    enum class AudioADCSource : uint8_t {
        Audio = ADC_MUXPOS_AIN8_gc,
        Mic = ADC_MUXPOS_AIN6_gc
    }; // AudioADCSource

    static_assert(AUDIO_ADC == 12, "Must be PC2, ADC1 input 8");
    static_assert(MIC == 10, "Must be PC0, ADC1 input 6");

    static void StartAudioADC(AudioADCSource channel) {
        AudioMin_ = 255;
        AudioMax_ = 0;
        RecordingIndex_ = 0;
        // select ADC channel to either MIC or audio ADC
        ADC1.MUXPOS  = static_cast<uint8_t>(channel);
        // enable and use 8bit resolution, freerun mode
        ADC1.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_8BIT_gc /*| ADC_FREERUN_bm */;
        // enable the interrupt
        ADC1.INTCTRL |= ADC_RESRDY_bm;
        // start the timer
        TCB0.CTRLA |= TCB_ENABLE_bm;
}

    static void StopAudioADC() {
        ADC1.CTRLA = 0;
        TCB0.CTRLA &= ~TCB_ENABLE_bm;
    }

    static bool AudioADCRunning() {
        return ADC1.CTRLA & ADC_ENABLE_bm;
    }

    static void UpdateAudioLights() {
        uint8_t v = AudioMax_ - AudioMin_;
        v = v < 32 ? 0 : v - 32;
        Neopixels_.showCenteredBar(v, 128, AccentColor_);
        AudioMin_ = 255;
        AudioMax_ = 0;
    }

    friend void ::ADC1_RESRDY_vect();
    //friend void ::TCB0_INT_vect();

    inline static volatile uint8_t AudioMin_;
    inline static volatile uint8_t AudioMax_;
    inline static volatile uint8_t AudioLightsMax_;

    inline static volatile uint8_t RecordingIndex_;
    inline static volatile uint8_t RecordingBuffer_[64];



//@}

}; // Player


ISR(RTC_PIT_vect) {
    //digitalWrite(AUDIO_SRC, HIGH);
    RTC.PITINTFLAGS = RTC_PI_bm;
    Player::Tick();
    //digitalWrite(AUDIO_SRC, LOW);
}

#define I2C_ADDRESS_MATCH (TWI_APIF_bm & TWI_AP_bm)
#define I2C_DATA_WRITE (TWI_DIF_bm & TWI_DIR_bm & TWI_RXACK_bm)
#define I2C_DATA_READ (TWI_DIF_bm)

void twi() {
//ISR(TWI0_TWIS_vect) {
    uint8_t status = TWI0.SSTATUS;
    // fastpath for sending data to master, in which case we just send new byte
    if (status & I2C_DATA_WRITE == I2C_DATA_WRITE) {

    }
    if (status & I2C_ADDRESS_MATCH == I2C_ADDRESS_MATCH) {
    }
}

ISR(ADC0_RESRDY_vect) {
    //digitalWrite(AUDIO_SRC, HIGH);
    uint16_t value = ADC0.RES / 64; // 64 sampling for better precission
    switch (ADC0.MUXPOS) {
        case ADC_MUXPOS_INTREF_gc: { // VCC Sense
            Player::State_.setVoltage(value);
            // switch the ADC to be ready to measure the temperature
            ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
            ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
            break;
        }
        case ADC_MUXPOS_TEMPSENSE_gc: { // tempreature sensor
            Player::State_.setTemp(value);
            // fallthrough to the default where we set the next measurement to be that of input voltage
        }
        default:
            // switch the ADC to be ready to measure the VCC
            ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
            ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
            break;
    }
    // start new conversion
    ADC0.COMMAND = ADC_STCONV_bm;
    //digitalWrite(AUDIO_SRC, LOW);    
}

ISR(ADC1_RESRDY_vect) {
    //digitalWrite(AUDIO_SRC, HIGH);
    //uint8_t value = ADC1.RES;
    Player::RecordingBuffer_[Player::RecordingIndex_++] = ADC1.RESL;
    Player::RecordingIndex_ &= 63;
    //digitalWrite(AUDIO_SRC, LOW);
}

/** The 8kHz interrupt for audio recording. 
 */
/*
ISR(TCB0_INT_vect) {
    //digitalWrite(AUDIO_SRC, HIGH);
    TCB0.INTFLAGS = TCB_CAPT_bm; // clear the flag
    ADC1.COMMAND = ADC_STCONV_bm;
    /*
    if (Player::RecordingCounter_ != 0) { // div by zero
        uint8_t value = Player::Recording_ / Player::RecordingCounter_;
        Player::Recording_ = 0;
        Player::RecordingCounter_ = 0;
        // whether this is audio, or microphone, update the audio bar levels
        if (value < Player::AudioMin_)
            Player::AudioMin_ = value; 
        if (value > Player::AudioMax_)
            Player::AudioMax_ = value;
        // if we are recording, store in the buffer
        Player::RecordingBuffer_[Player::RecordingIndex_];
        Player::RecordingIndex_ = (Player::RecordingIndex_ + 1) % 64;
        // TODO notify ESP to read    
    }
    * /
    //digitalWrite(AUDIO_SRC, LOW);
}
*/

void setup() {
    Player::Initialize();
}

void loop() {
    Player::Loop();
}

#endif // ARCH_ATTINY
