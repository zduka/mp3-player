#if (defined ARCH_ATTINY)

#if (F_CPU!=8000000UL)
#error "Only 8Mhz clock is supported"
#endif

#include <Arduino.h>

#include <avr/sleep.h>
#include <util/delay.h>

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
      NEOPIXEL -- (06) PB3   PC1 (11) -- AUDIO_SRC
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
extern "C" void TWI0_TWIS_vect(void) __attribute__((signal));
extern "C" void ADC1_RESRDY_vect(void) __attribute__((signal));

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
        InitializeI2C();
        // set transmit buffer to state and transmit length to state size
        I2C_TX_Buffer_ = pointer_cast<uint8_t*>(& State_);
        I2C_TX_Length_ = sizeof(State);     
        // enable control interrupts for buttons
        ControlBtn_.setInterrupt(ControlButtonChanged);
        VolumeBtn_.setInterrupt(VolumeButtonChanged);
        // allow voltages to stabilize when powered up
        delay(200);
        // from now on proceed identically to a wakeup
        Wakeup();
        //StartAudioADC(AudioADCSource::Mic);
    }

    static void Loop() {
        // if there is I2C message from ESP, process it
        if (Irq_.i2cRx)
            I2CReceive();
        if (ADC0.INTFLAGS & ADC_RESRDY_bm)
            VoltageAndTemperature();
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
                    // no need to disable interruts as esp is not running and so I2C can't be reading state at this point
                    State_.setControlDown(false);
                    State_.setVolumeDown(false);
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
        ADC0.COMMAND = ADC_STCONV_bm;
        // wait for the voltage measurement to finish
        while (! ADC0.INTFLAGS & ADC_RESRDY_bm) {
        }
        VoltageAndTemperature();
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
            // no need to disable interruts as esp is not running and so I2C can't be reading state at this point
            State_.clearButtonEvents();
            // if we are not longer sleeping, wakeup
            Wakeup();
        } while (Status_.sleep == true);
    }

    /** Flashes the strip three times with red as a critical battery indicator. 
     */
    static void CriticalBattery() {
        // disable I2C
        TWI0.SCTRLA = 0;
        TWI0.MCTRLA = 0;
        // set SDA SCL low so that ESP goes to sleep immediately
        pinMode(SDA, OUTPUT);
        pinMode(SCL, OUTPUT);
        digitalWrite(SDA, LOW);
        digitalWrite(SCL, LOW);
        // turn on peripheral voltage & flash the strip
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
        // initialize I2C back and go to sleep
        InitializeI2C();
        Status_.sleep = true;
    }

    static void VoltageAndTemperature() {
        uint16_t value = ADC0.RES / 64;
        switch (ADC0.MUXPOS) {
            case ADC_MUXPOS_INTREF_gc: { // VCC Sense
                cli();
                State_.setVoltage(value);
                sei();
                // switch the ADC to be ready to measure the temperature
                ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
                ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
                break;
            }
            case ADC_MUXPOS_TEMPSENSE_gc: { // tempreature sensor
                cli();
                State_.setTemp(value);
                sei();
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
    }


    friend void ::RTC_PIT_vect();

    inline volatile static struct {
        bool sleep : 1;
        bool tick : 1;
        bool secondTick : 1;
        uint8_t ticksCounter : 6; // 0..63
        bool recording : 1; // indicates that we are recording 
    } Status_;

    inline static State State_;

    inline static DateTime Time_;

/** \name Comms
 */
//@{
private:

    static void InitializeI2C() {
        // make sure that the pins are nout out - HW issue with the chip, will fail otherwise
        PORTB.OUTCLR = 0x03; // PB0, PB1
        // set the address and disable general call, disable second address and set no address mask (i.e. only the actual address will be responded to)
        TWI0.SADDR = AVR_I2C_ADDRESS << 1;
        TWI0.SADDRMASK = 0;
        // enable the TWI in slave mode, enable all interrupts
        TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm  | TWI_ENABLE_bm;
        // bus Error Detection circuitry needs Master enabled to work 
        // TODO not sure why we need it
        TWI0.MCTRLA = TWI_ENABLE_bm;   
    }

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

    static void I2CReceive() {
        // TODO check that numBytes <= message length or accept corrupted messages?
        switch (I2C_RX_Buffer_[0]) {
            case msg::PowerOff::Id: {
                // set sleep to true so that the sleep can be handled in main loop
                //Status_.sleep = true;
                break;
            }
            case msg::SetMode::Id: {
                auto msg = msg::At<msg::SetMode>(I2C_RX_Buffer_);
                cli();
                msg.applyTo(State_);
                sei();
                // sync the volume and control states
                Control_.setValues(State_.control(), State_.maxControl());
                Volume_.setValues(State_.volume(), State_.maxVolume());
                // sync the effect color for the night mode
                EffectColor_ = State_.nightLightColor(AccentColor_, MaxBrightness_);
                // set the audio source accordingly
                if (State_.mode() == Mode::Radio)
                    digitalWrite(AUDIO_SRC, HIGH);
                else
                    digitalWrite(AUDIO_SRC, LOW); 
                break;
            }
            case msg::SetWiFiStatus::Id: {
                auto msg = msg::At<msg::SetWiFiStatus>(I2C_RX_Buffer_);
                cli();
                msg.applyTo(State_);
                sei();
                break;
            }
            case msg::SetMP3Settings::Id: {
                auto msg = msg::At<msg::SetMP3Settings>(I2C_RX_Buffer_);
                cli();
                msg.applyTo(State_);
                sei();
                break;
            }
            case msg::SetRadioSettings::Id: {
                auto msg = msg::At<msg::SetRadioSettings>(I2C_RX_Buffer_);
                cli();
                msg.applyTo(State_);
                sei();
                break;
            }
            case msg::SetNightLightSettings::Id: {
                auto msg = msg::At<msg::SetNightLightSettings>(I2C_RX_Buffer_);
                cli();
                msg.applyTo(State_);
                sei();
                EffectColor_ = State_.nightLightColor(AccentColor_, MaxBrightness_);
                break;
            }
            case msg::SetAccentColor::Id: {
                auto msg = msg::At<msg::SetAccentColor>(I2C_RX_Buffer_);
                AccentColor_ = msg.color;
                break;
            }
            case msg::LightsBar::Id: {
                auto msg = msg::At<msg::LightsBar>(I2C_RX_Buffer_);
                SpecialLights_.showBar(msg.value, msg.max, msg.color);
                LightsCounter_ = msg.timeout;
                break;
            }
            case msg::LightsCenteredBar::Id: {
                auto msg = msg::At<msg::LightsCenteredBar>(I2C_RX_Buffer_);
                SpecialLights_.showCenteredBar(msg.value, msg.max, msg.color);
                LightsCounter_ = msg.timeout;
                break;
            }
            case msg::LightsPoint::Id: {
                auto msg = msg::At<msg::LightsPoint>(I2C_RX_Buffer_);
                SpecialLights_.showPoint(msg.value, msg.max, msg.color);
                LightsCounter_ = msg.timeout;
                break;
            }
            case msg::LightsColors::Id: {
                auto msg = msg::At<msg::LightsColors>(I2C_RX_Buffer_);
                for (int i = 0; i < 8; ++i)
                    SpecialLights_[i] = msg.colors[i];
                LightsCounter_ = msg.timeout;
                break;
            }
            case msg::StartRecording::Id: {
                Status_.recording = true;
                I2C_TX_Mode_ = I2C_TX_Mode::Recording;
                StartAudioADC(AudioADCSource::Mic);
                break;
            }
            case msg::StopRecording::Id: {
                Status_.recording = false;
                I2C_TX_Mode_ = I2C_TX_Mode::State;
                StopAudioADC();
                break;
            }
            case msg::GetTime::Id: {
                I2C_TX_Mode_ = I2C_TX_Mode::Time;
                break;
            }
        }
        Irq_.i2cRx = false;
        cli();
        I2C_RX_Offset_ = 0;
        sei();
    }

    friend void TWI0_TWIS_vect();


    static_assert(IRQ_MAX_DELAY < 128);
    inline volatile static struct {
        bool enabled : 1;
        uint8_t timer : 7; // 0..127
        bool i2cRx : 1; // indicates that I2C message has been received
    } Irq_;

    enum class I2C_TX_Mode : uint8_t {
        State,
        Recording,
        Time,
    }; 

    inline static volatile I2C_TX_Mode I2C_TX_Mode_ = I2C_TX_Mode::State;
    inline static uint8_t I2C_TX_Offset_ = 0;
    inline static uint8_t I2C_TX_Length_ = 0;
    inline static uint8_t * I2C_TX_Buffer_ = nullptr;

    inline static uint8_t I2C_RX_Offset_ = 0;
    inline static uint8_t I2C_RX_Buffer_[32];


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

    static void ControlValueChanged() { // IRQ
        Control_.poll();
        State_.setControl(Control_.value());
        SetIrq();
    }   

    static void VolumeValueChanged() { // IRQ
        Volume_.poll(); 
        State_.setVolume(Volume_.value());
        SetIrq();
    } 

    static void ControlButtonChanged() { // IRQ
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

    static void VolumeButtonChanged() { // IRQ
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
            NightLight(step);
        // display audio lights now
        // TODO remove the true flag and actually work this based on mode & audio lights state
        } else if (State_.audioLights() || true) {
            AudioLights(step);
        // otherwise the lights are off
        } else {
            Lights_.fill(Color::Black());
        }
        // update the actual neopixels buffer
        Neopixels_.moveTowardsReversed(Lights_, step);
        // draw over the bar notification lights - since we do this every tick, calculate the blinking brightness transition first
        AddNotifications();
        // and finally, update the neopixels
        Neopixels_.update();
    }

    static void AddNotifications() {
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
    }

    static void NightLight(uint8_t & step) {
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
    }

    static void AudioLights(uint8_t & step) {
        cli();
        uint8_t ri = RecordingWrite_;
        sei();
        uint8_t audioMin = 255;
        uint8_t audioMax = 0;
        for (uint8_t i = 0; i < 125; ++i) {
            uint8_t x = RecordingBuffer_[--ri];
            audioMin = (x < audioMin) ? x : audioMin;
            audioMax = (x > audioMax) ? x : audioMax;
        }
        uint8_t v = audioMax - audioMin;
        //  v = v < 32 ? 0 : v - 32;
        if (v > AudioLightsMax_)
            AudioLightsMax_ = v;
        Lights_.showCenteredBar(v, AudioLightsMax_, AccentColor_.withBrightness(MaxBrightness_));
        if (AudioLightsMax_ > 0 && Status_.ticksCounter % 4 == 0)
            --AudioLightsMax_;
    }

    /** Shows the given byte displayed on the neopixels strip and freezes. 
     
        The byte is displayed LSB first when looking from the front, or MSB first when looking from the back. 
     */
    static void ShowByte(uint8_t value, Color const & color) {
        Neopixels_[7] = (value & 1) ? color : Color::Black();
        Neopixels_[6] = (value & 2) ? color : Color::Black();
        Neopixels_[5] = (value & 4) ? color : Color::Black();
        Neopixels_[4] = (value & 8) ? color : Color::Black();
        Neopixels_[3] = (value & 16) ? color : Color::Black();
        Neopixels_[2] = (value & 32) ? color : Color::Black();
        Neopixels_[1] = (value & 64) ? color : Color::Black();
        Neopixels_[0] = (value & 128) ? color : Color::Black();
        Neopixels_.update();
        cli();
        while (true) { };
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
    inline static volatile uint8_t AudioLightsMax_;

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
        RecordingRead_ = 0;
        RecordingWrite_ = 0;
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

    friend void ::ADC1_RESRDY_vect();
    
    inline static volatile uint8_t RecordingWrite_ = 0;
    inline static volatile uint8_t RecordingRead_ = 0;
    inline static volatile uint8_t RecordingBuffer_[256];

//@}

}; // Player


ISR(RTC_PIT_vect) {
    digitalWrite(AUDIO_SRC, HIGH);
    RTC.PITINTFLAGS = RTC_PI_bm;
    Player::Tick();
    digitalWrite(AUDIO_SRC, LOW);
}

#define I2C_DATA_MASK (TWI_DIF_bm | TWI_DIR_bm) 
#define I2C_DATA_TX (TWI_DIF_bm | TWI_DIR_bm)
#define I2C_DATA_RX (TWI_DIF_bm)
#define I2C_START_MASK (TWI_APIF_bm | TWI_AP_bm | TWI_DIR_bm)
#define I2C_START_TX (TWI_APIF_bm | TWI_AP_bm | TWI_DIR_bm)
#define I2C_START_RX (TWI_APIF_bm | TWI_AP_bm)
#define I2C_STOP_MASK (TWI_APIF_bm | TWI_DIR_bm)
#define I2C_STOP_TX (TWI_APIF_bm | TWI_DIR_bm)
#define I2C_STOP_RX (TWI_APIF_bm)

ISR(TWI0_TWIS_vect) {
    //digitalWrite(AUDIO_SRC, HIGH);
    uint8_t status = TWI0.SSTATUS;
    // sending data to accepting master is on our fastpath. In this case depending on whether there is data to be send or not we send or don't the ack
    if ((status & I2C_DATA_MASK) == I2C_DATA_TX) {
        if (Player::I2C_TX_Offset_ < Player::I2C_TX_Length_) {
            // first byte of state is the events of the buttons, so once we send them, clear them so that we don't send them again
            if (Player::I2C_TX_Offset_ == 0 && Player::I2C_TX_Buffer_ == pointer_cast<uint8_t*>(& Player::State_))
                Player::State_.clearButtonEvents();
            TWI0.SDATA = Player::I2C_TX_Buffer_[Player::I2C_TX_Offset_++];
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
        // after the selected buffer has been sent, if the buffer is was not state, switch to state and continue sending as long as master wants more data
        } else if (Player::I2C_TX_Buffer_ != pointer_cast<uint8_t*>(& Player::State_)) {
            Player::I2C_TX_Buffer_ = pointer_cast<uint8_t *>(& Player::State_);
            Player::I2C_TX_Offset_ = 0;
            Player::I2C_TX_Length_ = sizeof(State);
            TWI0.SDATA = Player::I2C_TX_Buffer_[Player::I2C_TX_Offset_++];
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
        // otherwise don't send anything else, master will eventually send stop
        } else {
            TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
        }
    // a byte has been received from master. Store it and send either ACK if we can store more, or NACK if we can't store more
    } else if ((status & I2C_DATA_MASK) == I2C_DATA_RX) {
        Player::I2C_RX_Buffer_[Player::I2C_RX_Offset_++] = TWI0.SDATA;
        TWI0.SCTRLB = (Player::I2C_RX_Offset_ == 32) ? TWI_SCMD_COMPTRANS_gc : TWI_SCMD_RESPONSE_gc;
    // master requests slave to write data, send ACK if there is data to be transmitted, NACK if there is no data to send
    } else if ((status & I2C_START_MASK) == I2C_START_TX) {
        Player::ClearIrq();
        switch (Player::I2C_TX_Mode_) {
            case Player::I2C_TX_Mode::State: 
                Player::I2C_TX_Buffer_ = pointer_cast<uint8_t *>(& Player::State_);
                Player::I2C_TX_Offset_ = 0;
                Player::I2C_TX_Length_ = sizeof(State);
                // if in the middle of recording, switch back to recording mode for the next request
                if (Player::Status_.recording)
                    Player::I2C_TX_Mode_ = Player::I2C_TX_Mode::Recording;
                break;
            case Player::I2C_TX_Mode::Recording: 
                Player::I2C_TX_Buffer_ = pointer_cast<uint8_t *>(& Player::RecordingBuffer_) + Player::RecordingRead_;
                Player::I2C_TX_Offset_ = 0;
                Player::I2C_TX_Length_ = (static_cast<uint8_t>(Player::RecordingRead_ + 32) <= Player::RecordingWrite_) ? 32 : 0;
                break;
            // when sending time, set the buffer properly and revert back to the default TX mode
            case Player::I2C_TX_Mode::Time:
                Player::I2C_TX_Buffer_ = pointer_cast<uint8_t *>(& Player::Time_);
                Player::I2C_TX_Offset_ = 0;
                Player::I2C_TX_Length_ = sizeof(DateTime);
                if (Player::Status_.recording)
                    Player::I2C_TX_Mode_ = Player::I2C_TX_Mode::Recording;
                else
                    Player::I2C_TX_Mode_ = Player::I2C_TX_Mode::State;
                break;
        }
        if (Player::I2C_TX_Offset_ < Player::I2C_TX_Length_) {
            TWI0.SCTRLB = TWI_ACKACT_ACK_gc + TWI_SCMD_RESPONSE_gc;
        } else {
            TWI0.SCTRLB = TWI_ACKACT_NACK_gc + TWI_SCMD_COMPTRANS_gc;
        }
    // master requests to write data itself. ACK if the buffer is empty (we do not support multiple commands in same buffer), NACK otherwise.
    } else if ((status & I2C_START_MASK) == I2C_START_RX) {
        if (Player::I2C_RX_Offset_ == 0)
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
        else
            TWI0.SCTRLB = TWI_ACKACT_NACK_gc;
    // when a transmission finishes we must see if another transmission required and re-raise the irq flag for ESP. While recording this means we need to see if there is another 32 bytes available yet
    } else if ((status & I2C_STOP_MASK) == I2C_STOP_TX) {
        if (Player::Status_.recording) {
            if (Player::I2C_TX_Offset_ == 32) // we only transmit 32byte chunks, anything less will be retransmitted as it was likely an error
                Player::RecordingRead_ += 32;
            // if the read and write offsets are into the same 32 bit partition, then the flag will be raised when more samples are available, otherwise raise the flag now
            if (Player::RecordingRead_ /32 != Player::RecordingWrite_ / 32)
                Player::SetIrq();
        }
        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
    } else if ((status & I2C_STOP_MASK) == I2C_STOP_RX) {
        Player::Irq_.i2cRx = true;
        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
    } else {
        Player::ShowByte(status, Color::Red());
    }
    //digitalWrite(AUDIO_SRC, LOW);
}

ISR(ADC1_RESRDY_vect) {
    //digitalWrite(AUDIO_SRC, HIGH);
    Player::RecordingBuffer_[Player::RecordingWrite_++] = ADC1.RESL;
    if (Player::RecordingWrite_ % 32 == 0 && Player::Status_.recording)
        Player::SetIrq();
    //digitalWrite(AUDIO_SRC, LOW);
}

void setup() {
    Player::Initialize();
}

void loop() {
    Player::Loop();
}

#endif // ARCH_ATTINY
