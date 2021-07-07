#if (defined ARCH_ATTINY)

#if (F_CPU!=8000000UL)
#error "Only 8Mhz clock is supported"
#endif


#include <avr/sleep.h>
#include <util/delay.h>
#include <Wire.h>

#include "state.h"
#include "comms.h"
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
      NEOPIXEL -- (06) PB3   PC1 (11) -- 
       AVR_IRQ -- (07) PB2   PC0 (10) -- MIC
           SDA -- (08) PB1   PB0 (09) -- SCL

TODO change AVR_IRQ to PB5 and set PB2 for TX for serial, this would make debugging less painful
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
        // configure the real time clock
        RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; // select internal oscillator
        RTC.PITINTCTRL |= RTC_PI_bm; // enable the interrupt
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
        // set ADC1 settings (mic & audio) - clkdiv by 4, internal voltage reference
        ADC1.CTRLC = ADC_PRESC_DIV4_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm; // 2mhz
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
        
        // from now on proceed identically to a wakeup
        Wakeup();
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
        }
    }

private:

    /** Initialized when the chip wakes up into normal operation. 
     */
    static void Wakeup() {
        // enable the RTC interrupt to 1/32th of a second while awake
        while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
        RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc + RTC_PITEN_bm;
        // start ADC0 to measure the VCC and wait for the first measurement to be done
        ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
        ADC0.INTCTRL |= ADC_RESRDY_bm;
        ADC0.COMMAND = ADC_STCONV_bm;
        // wait for the voltage measurement to finish
        //while (ADC0.MUXPOS == ADC_MUXPOS_INTREF_gc) {
        //}
        // TODO deal with the voltage value appropriately
        
        // enable interrupts for rotary encoders
        Control_.setInterrupt(ControlValueChanged);
        Volume_.setInterrupt(VolumeValueChanged);
        // turn on ESP and neopixels
        // TODO have this in a method so that it can be called from reset function as well? 
        pinMode(DCDC_PWR, OUTPUT);
        digitalWrite(DCDC_PWR, LOW);
        delay(50);
        // TODO flash neopixels for wakeup
    }

    static void Sleep() {
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
    }

    friend void ::RTC_PIT_vect();

    friend void ::ADC0_RESRDY_vect();

    inline volatile static struct {
        bool sleep : 1;
        bool idle : 1;
        bool tick : 1;
        bool secondTick : 1;
        uint8_t ticksCounter : 5; // 0..31
        uint8_t sleepCountdown: 10; // 0..1023
    } Status_;

    inline static volatile State State_;

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
        switch (I2CSource_) {
            case I2CSource::State:
                Wire.write(pointer_cast<uint8_t*>(& State_), sizeof (State));
                State_.clearEvents();
                ClearIrq();
        }
    }

    static void I2CReceive(int numBytes) {
        // TODO check that numBytes <=32
        Wire.readBytes(pointer_cast<uint8_t*>(& Buffer_), numBytes);
        switch (Buffer_[0]) {
            case Message::SetVolume::Id: {
                auto msg = Message::SetVolume::At(Buffer_);
            }
            case Message::SetControl::Id: {
            
            }
        }
    }

    enum class I2CSource : uint8_t {
        State
    };

    static_assert(IRQ_MAX_DELAY < 128);
    inline volatile static struct {
        bool enabled : 1;
        bool timer : 7; // 0..127
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
                RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc + RTC_PITEN_bm;     
            }
        } else {
            State_.setControlDown(false);
            if (ControlBtnCounter_ == 0) {
                State_.setControlLongPress();
                // wake up from sleep if sleeping
                Status_.sleep = false;
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
                RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc + RTC_PITEN_bm;     
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

    static void LightsTick() {




    }

    inline static NeopixelStrip<NEOPIXEL, 8> Neopixels_;
    inline static Neopixel AccentColor_ = Neopixel::White();
    inline static uint8_t MaxBrightness_ = 255;

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
        // select ADC channel to either MIC or audio ADC
        ADC1.MUXPOS  = static_cast<uint8_t>(channel);
        // enable and use 8bit resolution, freerun mode
        ADC1.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_8BIT_gc | ADC_FREERUN_bm;
        // enable the interrupt
        ADC1.INTCTRL |= ADC_RESRDY_bm;
        // and start the conversion
        ADC1.COMMAND = ADC_STCONV_bm;        
    }

    static void StopAudioADC() {
        ADC1.CTRLA = 0;
    }

    static bool AudioADCRunning() {
        return ADC1.CTRLA & ADC_ENABLE_bm;
    }

    static void UpdateAudioLights() {
        uint8_t v = AudioMax_ - AudioMin_;
        v = v < 32 ? 0 : v - 32;
        Neopixels_.showCenteredBar(v, 128, AUDIO_COLOR.withBrightness(MaxBrightness_));
        AudioMin_ = 255;
        AudioMax_ = 0;
    }

    friend void ::ADC1_RESRDY_vect();

    inline static volatile uint8_t AudioMin_;
    inline static volatile uint8_t AudioMax_;

//@}

}; // Player


ISR(RTC_PIT_vect) {
    RTC.PITINTFLAGS = RTC_PI_bm;
    // determine whether we have 1 second or 1/32th second ticks and call the appropriate functions
    if (! Player::Status_.sleep || Player::State_.controlDown() || Player::State_.volumeDown()) {
        Player::Status_.tick = true;
        // if buttons are down, decrease their long press ticks
        if (Player::ControlBtnCounter_ > 0)
            --Player::ControlBtnCounter_;
        if (Player::VolumeBtnCounter_ > 0)
            --Player::VolumeBtnCounter_;
        // if this is the 32th tick, continue to second tick, otherwise return now            
        if (Player::Status_.ticksCounter-- != 0)
            return;
    } 
    // the one-second tick
    Player::Status_.secondTick = true;
    Player::Time_.secondTick();
    // TODO alarm & stuff
}

ISR(ADC0_RESRDY_vect) {
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
}

ISR(ADC1_RESRDY_vect) {
    uint8_t value = ADC1.RES;
    // whether this is audio, or microphone, update the audio bar levels
    if (value < Player::AudioMin_)
        Player::AudioMin_ = value; 
    if (value > Player::AudioMax_)
        Player::AudioMax_ = value;
}

void setup() {
    Player::Initialize();
}

void loop() {
    Player::Loop();
}


#ifdef HAHA

#define AUDIO_ESP HIGH
#define AUDIO_RADIO LOW

extern "C" void RTC_PIT_vect(void) __attribute__((signal));
extern "C" void ADC0_RESRDY_vect(void) __attribute__((signal));
extern "C" void ADC1_RESRDY_vect(void) __attribute__((signal));

/** The MP3 Player Device Controller

    # RTC

    The internal RTC in the attiny is used to keep a semi-precise track of time. When the chip is on, a tick is generated every 1/32th of second so that it can be used to drive neopixels, etc. When the device sleeps, a one second tick is used to save battery and only the time is updated and alarm checked. 
 */
class AVRPlayer {
public:

    /** Setup is executed when powered on. 

        Technically, this can only happen after a reset, or after BOD when voltage gets high again due to charging, or new battery. As such, we can simply assume enough battery is present and power stuff without checking voltage. 
     */
    void setup() {
        // enable the AVR_IRQ as input
        pinMode(AVR_IRQ, INPUT);
        // disable 3v, audio and neopixel rails
        pinMode(DCDC_PWR, INPUT);
        // configure the real time clock
        RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; // select internal oscillator
        RTC.PITINTCTRL |= RTC_PI_bm; // enable the interrupt
        // set audio source to esp8266
        pinMode(AUDIO_SRC, OUTPUT);
        digitalWrite(AUDIO_SRC, AUDIO_ESP);
        // enable the ADC inputs (audio, mic, voltage), disable the digital input buffers and pull-up resistors
        // NOTE this requires that the pins stay the same
        static_assert(AUDIO_ADC == 12, "Must be PC2"); // ADC1 input 8
        PORTC.PIN2CTRL &= ~PORT_ISC_gm;
        PORTC.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;
        PORTC.PIN2CTRL &= ~PORT_PULLUPEN_bm;
        static_assert(MIC == 10, "Must be PC0"); // ADC1 input 6
        PORTC.PIN0CTRL &= ~PORT_ISC_gm;
        PORTC.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
        PORTC.PIN0CTRL &= ~PORT_PULLUPEN_bm;
        static_assert(HEADPHONES == 14, "Must be PA1"); // ADC0 input 1
        PORTA.PIN1CTRL &= ~PORT_ISC_gm;
        PORTA.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
        PORTA.PIN1CTRL &= ~PORT_PULLUPEN_bm;

        // set sleep to full power down and enable sleep feature
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();         
        // setup ADC0 and ADC1 settings
        // set reference to 1.1V
        VREF.CTRLA &= ~ VREF_ADC0REFSEL_gm;
        VREF.CTRLA |= VREF_ADC0REFSEL_1V1_gc;
        VREF.CTRLC &= ~ VREF_ADC1REFSEL_gm;
        VREF.CTRLC |= VREF_ADC1REFSEL_1V1_gc;
        // clkdiv by 4, internal voltage reference
        ADC1.CTRLC = ADC_PRESC_DIV4_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm; // 2mhz
        // delay 32us and sampctrl of 32 us for the temperature sensor, do averaging over 16 values
        ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
        ADC0.CTRLB = ADC_SAMPNUM_ACC64_gc;
        ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
        ADC0.CTRLD = ADC_INITDLY_DLY32_gc;
        ADC0.SAMPCTRL = 31;
        // start the vcc & temp & headphones sensing, enable interrupt and measure the VCC once more
        ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
        ADC0.INTCTRL |= ADC_RESRDY_bm;
        ADC0.COMMAND = ADC_STCONV_bm;
        // enable I2C slave
        Wire.begin(AVR_I2C_ADDRESS);
        Wire.onRequest(I2CSendEventTrampoline);
        Wire.onReceive(I2CReceiveEventTrampoline);
        // attach the interrupts
        volume_.setInterrupt(VolumeChangedTrampoline);
        control_.setInterrupt(ControlChangedTrampoline);
        volumeButton_.setInterrupt(VolumeButtonChangedTrampoline);
        controlButton_.setInterrupt(ControlButtonChangedTrampoline);
        // and continue as if waking up from sleep
        wakeup();
    }
    
    void loop() {
        if (rtcTick()) {
            if (irq_.enabled && --irq_.timer == 0) {
                resetEsp();
            } else {
                // update the lights
                lightsTick();
            }
        }
        if (rtcTickSecond()) {
            if (state_.idle() && sleep_.countdown > 0) {
                if (--sleep_.countdown == 0)
                    sleep();
            }
        }
    }

private:

    static constexpr unsigned AVR_ACTIVE = 0;
    static constexpr unsigned AVR_SLEEPING = 1;
    static constexpr unsigned AVR_WAKING = 2;

    /** The sleep state and countdown. 
     */
    volatile struct {
        unsigned state : 2;
        unsigned countdown : 6;        
    } sleep_;


    void setIdle(bool value) {
        state_.setIdle(value);
        if (value) {
            sleep_.countdown = 60;
        }
    }
    
    /** Enters sleep mode. 

        
     */
    void sleep() {
        neopixels_.showBar(8,8,Neopixel::Red());
        neopixels_.sync();
        delay(300);
        neopixels_.setAll(Neopixel::Black());
        neopixels_.sync();
        // set audio src to 0 so that we don't bleed voltage
        digitalWrite(AUDIO_SRC, LOW);
        // turn of power to 3v3 and 5v
        powerOff();
        // going to sleep
        sleep_.state = AVR_SLEEPING;
        // change rtc period to 1 second
        RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc + RTC_PITEN_bm; // enable PTI and set the tick to 1/32th of second
        // disable interrupts for rotary encoders (we only keep the buttons)
        // TODO
        

        // since we will keep receiving the RTC interrupts every 1 second, we'll sleep immediately afterwards unless it is a button interrupt which clears the sleep flag
        while (sleep_.state != AVR_ACTIVE)
            sleep_cpu();
        // here we wake up
        wakeup();
    }

    void wakeup() {
        RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc + RTC_PITEN_bm; // enable PTI and set the tick to 1/32th of second
        // wake-up to non-idle mode
        setIdle(false);
        powerOn();
        // if current mode is radio, make sure to to enable the source
        if (state_.mode() == State::Mode::Radio)
            digitalWrite(AUDIO_SRC, AUDIO_RADIO);
        else
            digitalWrite(AUDIO_SRC, AUDIO_ESP);
        delay(100);
        neopixels_.showBar(8,8,Neopixel::Green());
        neopixels_.sync();
        delay(300);
        neopixels_.setAll(Neopixel::Black());
        neopixels_.sync();
    }

    void setMode(State::Mode mode) {
        state_.setMode(mode);
        switch (mode) {
            case State::Mode::MP3:
                enableAudioADC();
                digitalWrite(AUDIO_SRC, AUDIO_ESP);
                controlColor_ = state_.mp3PlaylistSelection() ? MP3_PLAYLIST_COLOR : MP3_TRACK_COLOR;
                break;
            case State::Mode::Radio:
                enableAudioADC();
                digitalWrite(AUDIO_SRC, AUDIO_RADIO);
                controlColor_ = state_.radioManualTuning() ? RADIO_FREQUENCY_COLOR : RADIO_STATION_COLOR;
                break;
        }
    }

    
    void setVolume(uint8_t value) {
        if (value != state_.audioVolume()) 
            state_.setAudioVolume(value);
        neopixels_.showBar(state_.audioVolume(), 15, VOLUME_COLOR.withBrightness(brightness_));
        specialLights_ = SPECIAL_LIGHTS_TIMEOUT;
    }

    void setControl(uint16_t value) {
        if (value != state_.control()) 
            state_.setControl(value);
        neopixels_.showPoint(state_.control(), control_.maxValue() - 1, controlColor_.withBrightness(brightness_));
        specialLights_ = SPECIAL_LIGHTS_TIMEOUT;
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
        pinMode(DCDC_PWR,OUTPUT);
        digitalWrite(DCDC_PWR, LOW);
        state_.state_ |= State::STATE_DCDC_POWER; 
        delay(50);
    }

    void powerOff() {
        pinMode(DCDC_PWR, INPUT);
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
        pinMode(DCDC_PWR,INPUT);
        delay(500);
        pinMode(DCDC_PWR,OUTPUT);
        digitalWrite(DCDC_PWR, LOW);
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
        if (sleep_.state != AVR_SLEEPING) {
            rtcTick_.tick = 1;
            if (++rtcTick_.counter == 0) {
                rtcTick_.tickSecond = 1;
                clock_.secondTick();
            }
            // if the buttons are pressed, decrement their down ticks so that long presses can be determined, if the long press is the wakeup press clear the state for given button so that it won't register
            if (volBtnDownTicks_ > 0)
                if (--volBtnDownTicks_ == 0 && sleep_.state == AVR_WAKING) {
                    state_.state_ &= ~State::STATE_VOL_BTN;
                    sleep_.state = AVR_ACTIVE;
                }
            if (ctrlBtnDownTicks_ > 0)
                if (--ctrlBtnDownTicks_ == 0 && sleep_.state == AVR_WAKING) {
                    state_.state_ &= ~State::STATE_CTRL_BTN;
                    sleep_.state = AVR_ACTIVE;
                }
        } else {
            clock_.secondTick();
        }
    }


    /** \name Lights

     */
    //@{
    NeopixelStrip<NEOPIXEL, 8> neopixels_;
    uint8_t specialLights_ = 0;
    Neopixel accentColor_ = Neopixel::White();
    Neopixel controlColor_;
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
        volume_.poll();
        setVolume(volume_.value());
        setIrq();
    }

    void controlChanged() {
        control_.poll();
        setControl(control_.value());
        setIrq();
    }

    void volumeButtonChanged() {
        volumeButton_.poll();
        if (volumeButton_.pressed()) {
            if (sleep_.state == AVR_SLEEPING) {
                // switch RTC to every 1/32th second
                RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc + RTC_PITEN_bm; 
                sleep_.state = AVR_WAKING;
            } 
            volBtnDownTicks_ = BUTTON_LONG_PRESS_TICKS;
            state_.state_ |= State::STATE_VOL_BTN;
        // if this was waking-up press, it has been cleared so that it won't register
        } else if (state_.volumeButtonDown()){
            state_.state_ &= ~State::STATE_VOL_BTN;
            // go back to sleep if the threshold has not been reached
            if (sleep_.state == AVR_WAKING) {
                sleep_.state = AVR_SLEEPING;
                // switch RTC to once a second
                RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc + RTC_PITEN_bm;
                return;
            }
            state_.events_ |= (volBtnDownTicks_ == 0) ? State::EVENT_VOL_LONG_PRESS : State::EVENT_VOL_PRESS;
            neopixels_.setAll(Neopixel::Black());
            setIrq();
        }
    }

    void controlButtonChanged() {
        controlButton_.poll();
        if (controlButton_.pressed()) {
            if (sleep_.state == AVR_SLEEPING) {
                // switch RTC to every 1/32th second
                RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc + RTC_PITEN_bm; 
                sleep_.state = AVR_WAKING;
            } 
            ctrlBtnDownTicks_ = BUTTON_LONG_PRESS_TICKS;
            state_.state_ |= State::STATE_CTRL_BTN;
        // if this was waking-up press, it has been cleared so that it won't register
        } else if (state_.controlButtonDown()){
            state_.state_ &= ~State::STATE_CTRL_BTN;
            // go back to sleep if the threshold has not been reached
            if (sleep_.state == AVR_WAKING) {
                sleep_.state = AVR_SLEEPING;
                // switch RTC to once a second
                RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc + RTC_PITEN_bm;
                return;
            }
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
                controlColor_ = state_.radioManualTuning() ? RADIO_FREQUENCY_COLOR : RADIO_STATION_COLOR;
                break;
            }
                
            case Command::SetMP3State::Id: {
                Command::SetMP3State * cmd = pointer_cast<Command::SetMP3State*>(& cmdBuffer_[1]);
                state_.setMp3PlaylistId(cmd->playlist);
                state_.setMp3TrackId(cmd->track);
                state_.setMp3PlaylistSelection(cmd->playlistSelection);
                controlColor_ = state_.mp3PlaylistSelection() ? MP3_PLAYLIST_COLOR : MP3_TRACK_COLOR;
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

    /** \name ADC0 - Voltage, temperature and headphones
     */
    //@{

    void setVoltageFromADC(uint16_t adc) {
        // kind of complicated so that we stay in the 16bit unsigned, which 110 * 1024 won't give us
        adc = 110 * 512 / adc;
        adc = adc * 2;
        // convert adc to voltage times 100, i.e. in range 0..110, but anything below 0.55 volts gets compressed to 0
        //adc = (adc > 512) ? ((adc - 512) * 110 / 1023) + 55 : 0;
        // update to real voltage using the voltage divider
        //adc = adc * (VCC_DIVIDER_RES1 + VCC_DIVIDER_RES2) / VCC_DIVIDER_RES2;
        state_.setVoltage(adc);
    }

    void setTemperatureFromADC(uint16_t temp) {
        int8_t sigrow_offset = SIGROW.TEMPSENSE1; // Read signed value from signature row
        uint8_t sigrow_gain = SIGROW.TEMPSENSE0;
        temp -= sigrow_offset >> 2;
        temp *= (sigrow_gain >> 2); // so that we won't overflow 16bit value
        state_.setTemperature(temp);
    }

    void adc0ResultReady(uint16_t adc) {
        switch (ADC0.MUXPOS) {
            case ADC_MUXPOS_INTREF_gc: // VCC sense
                // switch the ADC to internal reference which will be used for other readings
                ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
                ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
                setVoltageFromADC(adc);
                break;
            case ADC_MUXPOS_AIN1_gc: { // headphones
                ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
                bool headphones = adc < 512;
                if (headphones != state_.audioHeadphones()) {
                    state_.setAudioHeadphones(headphones);
                    setIrq();
                }
                break;
            }
            case ADC_MUXPOS_TEMPSENSE_gc: // temperature
                // switch the ADC to VDD reference which will be used 
                ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
                ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
                setTemperatureFromADC(adc);
                break;
            default: // invalid muxpoint, start VCC_SENSE
                ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
                ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
                break;
        }
        ADC0.COMMAND = ADC_STCONV_bm;
    }

    //@}

    

    /** \name Audio output and mic readouts. 
     */
    //@{


    void enableAudioADC() {
        audioMin_ = 255;
        audioMax_ = 0;
        audioSamples_ = 0;
        // select ADC channel to AUDIO_ADC pin
        ADC1.MUXPOS  = ADC_MUXPOS_AIN8_gc;
        // enable and use 8bit resolution, freerun mode
        ADC1.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_8BIT_gc | ADC_FREERUN_bm;
        // enable the interrupt
        ADC1.INTCTRL |= ADC_RESRDY_bm;
        // and start the conversion
        ADC1.COMMAND = ADC_STCONV_bm;        
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
    player.adc0ResultReady(ADC0.RES / 64); // 64 sampling 
}

ISR(ADC1_RESRDY_vect) {
    player.addAudioReading(ADC1.RES); 
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

void loop() {
    player.loop();
}

#endif

#endif