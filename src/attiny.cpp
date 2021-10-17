#if (defined ARCH_ATTINY)

#if (F_CPU!=8000000UL)
#error "Only 8Mhz clock is supported"
#endif

#include <Arduino.h>

#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>

#if (defined TEST_RADIO)
#include <radio.h>
#include <RDA5807M.h>
#endif

#include "config.h"
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
      NEOPIXEL -- (04) PB5   PC3 (13) -- CTRL_B
      CHARGING -- (05) PB4   PC2 (12) -- AUDIO_ADC
       AVR_IRQ -- (06) PB3   PC1 (11) -- AUDIO_SRC
               -- (07) PB2   PC0 (10) -- MIC
           SDA -- (08) PB1   PB0 (09) -- SCL

 */

#define DCDC_PWR 3
#define NEOPIXEL 4
#define CTRL_A 16
#define CTRL_B 13
#define CTRL_BTN 15
#define VOL_A 2
#define VOL_B 0
#define VOL_BTN 1
#define AVR_IRQ 6
#define AUDIO_SRC 11
#define AUDIO_ADC 12
#define HEADPHONES 14
#define MIC 10


#define AUDIO_SRC_ESP HIGH
#define AUDIO_SRC_RADIO LOW

#define NOTIFICATION_LOW_BATTERY_COLOR Color::Red()
#define NOTIFICATION_NEW_MESSAGE_COLOR Color::Green()
#define NOTIFICATION_WIFI_CONNECTING_COLOR Color::Blue()
#define NOTIFICATION_WIFI_AP_COLOR Color::Cyan()

extern "C" void RTC_PIT_vect(void) __attribute__((signal));
extern "C" void TWI0_TWIS_vect(void) __attribute__((signal));
extern "C" void ADC1_RESRDY_vect(void) __attribute__((signal));

#if (defined TEST_NEOPIXEL)
void setup() {
    NeopixelStrip<NEOPIXEL, 8> neopixels;
    pinMode(DCDC_PWR, OUTPUT);
    digitalWrite(DCDC_PWR, LOW);
    delay(50);
    neopixels[0] = Color::Red();
    neopixels[1] = Color::Green();
    neopixels[2] = Color::Blue();
    neopixels[3] = Color::Purple();
    neopixels[4] = Color::Yellow();
    neopixels[5] = Color::Cyan();
    neopixels[6] = Color::White();
    neopixels[7] = Color::Red();
    neopixels.update();
}
void loop() { 
}
#elif (defined TEST_RADIO)
void setup() {
    pinMode(DCDC_PWR, OUTPUT);
    digitalWrite(DCDC_PWR, LOW);
    pinMode(AUDIO_SRC, OUTPUT);
    digitalWrite(AUDIO_SRC, LOW);
    pinMode(HEADPHONES, OUTPUT);
    digitalWrite(HEADPHONES, LOW);

    delay(50);
    Wire.begin();
    RDA5807M radio;
    radio.init();
    radio.setMono(true);
    radio.setVolume(1);
    radio.setBandFrequency(RADIO_BAND_FM, 9370);
}
void loop() { 
}
#else 
class Player {
public:
    static void initialize() {
        // disable power to other systems
        pinMode(DCDC_PWR, INPUT);
        // enable the AVR_IRQ as input so that we can observe if ESP wants something
        pinMode(AVR_IRQ, INPUT);
        // headphones are input pin as well
        pinMode(HEADPHONES, INPUT);
        // audio source is output, set it low by default
        pinMode(AUDIO_SRC, OUTPUT);
        digitalWrite(AUDIO_SRC, LOW);
        // set sleep to full power down and enable sleep feature
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();     
        // because this is poweron, delay a bit so that all voltages in the circuit can stabilize a bit, 200ms should do the trick
        delay(200); 
        
        initializeRTC();

        // Initialize ADC0
        initializeMeasurementsADC();

        initializeAudioCapture();
        


        // enable control interrupts for buttons
        controlBtn_.setInterrupt(controlButtonChanged);
        volumeBtn_.setInterrupt(volumeButtonChanged);
        // initialize comms
        initializeI2C();


        wakeup();
    }

    static void loop() {
        if (status_.tick) {
            // it is possible that between the irq check and the countdown check the irq will be cleared, however the second check would then only pass if the irq countdown was at its end and therefore the reset is ok
            if (status_.irq && irqCountdown_ == 0)
                resetESP();
            // make sure we have a light show
            lightsTick();
            cli();
            status_.tick = false;
            sei();
        }
        if (status_.i2cRxReady)
            processCommand();
        // if the measurement (headphones, voltage, temp, ...) is ready, measure
        if (ADC0.INTFLAGS & ADC_RESRDY_bm)
            measurementsADCReady();
    }

private:

/** \name ATTiny Management. 
 */
//@{

    /** Initializes the real-time clock. 
     
     */
    static void initializeRTC() {
        // configure the real time clock
        RTC.CLKSEL = RTC_CLKSEL_INT1K_gc; // select internal oscillator divided by 32
        RTC.PITINTCTRL |= RTC_PI_bm; // enable the interrupt
    }

    /** Initializes ADC0 used to take voltage & temperature measurements. 
     */
    static void initializeMeasurementsADC() {
        // setup ADC voltage references to 1.1V (internal)
        VREF.CTRLA &= ~ VREF_ADC0REFSEL_gm;
        VREF.CTRLA |= VREF_ADC0REFSEL_1V1_gc;
        // set ADC0 settings (vcc, temperature)
        // delay 32us and sampctrl of 32 us for the temperature sensor, do averaging over 64 values
        ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc;
        ADC0.CTRLB = ADC_SAMPNUM_ACC64_gc;
        ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
        ADC0.CTRLD = ADC_INITDLY_DLY32_gc;
        ADC0.SAMPCTRL = 31;
    }

    /** Puts the AVR to sleep. 
     */
    static void sleep() {
        do { // wrapped in a loop to account to allow wakeup to re-enter sleep immediately (such as when low volatge, etc.)
            // disable rotary encoder interrupts so that they do not wake us from sleep
            control_.clearInterrupt();
            volume_.clearInterrupt();
            // turn off neopixels and esp
            peripheralPowerOff();
            // enable RTC interrupt every second so that the time can be kept
            while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
            RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc + RTC_PITEN_bm;
            // enter the sleep mode. Upon wakeup, go to sleep immediately for as long as the sleep mode is on (we wake up every second to increment the clock and when buttons are pressed as well)
            status_.sleep = true;
            while (status_.sleep) {
                sleep_cpu();
            }
            // clear the button events that led to the wakeup
            // no need to disable interruts as esp is not running and so I2C can't be reading state at this point
            state_.clearEvents();
            // if we are not longer sleeping, wakeup
            wakeup();
        } while (status_.sleep);
    }

    static void wakeup() {
        // enable RTC interrupt every 1/64th of a second
        while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
        RTC.PITCTRLA = RTC_PERIOD_CYC16_gc + RTC_PITEN_bm;
        // start ADC0 to measure the VCC and wait for the first measurement to be done
        ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
        ADC0.COMMAND = ADC_STCONV_bm;
        // wait for the voltage measurement to finish
        while (! ADC0.INTFLAGS & ADC_RESRDY_bm) {
        }
        measurementsADCReady();
        if (ex_.measurements.vcc <= BATTERY_CRITICAL_VCC) {
            criticalBatteryWarning();
            status_.sleep = true;
        } else {
            // enable interrupts for rotary encoders
            control_.setInterrupt(controlKnobChanged);
            volume_.setInterrupt(volumeKnobChanged);
            // enable power to neopixels, esp8266 and other circuits
            peripheralPowerOn();
            // TODO show the wakeup progressbar *if* not in silent mode
            neopixels_.showBar(1,8, DEFAULT_COLOR.withBrightness(ex_.settings.maxBrightness));
            neopixels_.update();
            // TODO silent mode

            // disable idle mode when waking up
            // TODO perhaps not do this for silent mode? 
            state_.setIdle(false);
        }
    }

    /** Shows a critical battery warning before going back to sleep. 
     
        Powers on peripherals with both SDA and SCL held low, which puts ESP immediately in deep sleep and then flashes the full LED strip red three times. 
     */
    static void criticalBatteryWarning(uint8_t brightness = 255) {
        // disable I2C
        TWI0.SCTRLA = 0;
        TWI0.MCTRLA = 0;
        // set SDA SCL low so that ESP goes to sleep immediately
        pinMode(SDA, OUTPUT);
        pinMode(SCL, OUTPUT);
        digitalWrite(SDA, LOW);
        digitalWrite(SCL, LOW);
        // turn on peripheral voltage & flash the strip
        peripheralPowerOn();
        neopixels_.fill(Color::Red().withBrightness(brightness));
        neopixels_.update();
        delay(50);
        neopixels_.fill(Color::Black());
        neopixels_.update();
        delay(100);
        neopixels_.fill(Color::Red().withBrightness(brightness));
        neopixels_.update();
        delay(50);
        neopixels_.fill(Color::Black());
        neopixels_.update();
        delay(100);
        neopixels_.fill(Color::Red().withBrightness(brightness));
        neopixels_.update();
        delay(50);
        // this also makes the strip look black;)
        peripheralPowerOff(); 
        // initialize I2C back so that we are ready once the power stabilizes
        initializeI2C();
    }

    static void peripheralPowerOn() {
        pinMode(DCDC_PWR, OUTPUT);
        digitalWrite(DCDC_PWR, LOW);
        // add a delay so that the capacitor can be charged and voltage stabilized
        delay(50);
    }

    static void peripheralPowerOff() {
        pinMode(DCDC_PWR, INPUT);
    }

    /** Processes the voltage, temperature and headphones measurements. 
     
        Sets the appropriate flags in state. 
     */
    static void measurementsADCReady() {
        uint16_t value = ADC0.RES / 64;
        switch (ADC0.MUXPOS) {
            case ADC_MUXPOS_INTREF_gc: { // VCC Sense
                // convert value to voltage * 100, using the x = 1024 / value * 110 equation. Multiply by 512 only, then divide and multiply by two so that we always stay within uint16_t
                value = 110 * 512 / value;
                value = value * 2;
                cli();
                ex_.measurements.vcc = value;
                sei();
                // switch the ADC to be ready to measure the temperature
                ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
                ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
                break;
            }
            case ADC_MUXPOS_TEMPSENSE_gc: { // tempreature sensor
                // TODO calculate the value properly
                cli();
                ex_.measurements.temp = value;
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

//@}

/** \name Controls
 */
//@{

    /** \name Button Change Events  [ISR, sleep]

        Control and volume buttons are handled inside an interrupt handler and can run even when the attiny sleeps, which is handled by a special function. 

        During normal operation, short and long press of a button, or a double long press of both buttons are detected and raised as events.   
     */
    //@{
    static void controlButtonChanged() {
        controlBtn_.poll();
        if (status_.sleep) {
            checkButtonWakeup(controlBtn_.pressed());
        } else if (controlBtn_.pressed()) {
            longPressCounter_ = BUTTON_LONG_PRESS_TICKS;
            state_.setControlButtonDown();
            setIrq();
        } else if (state_.controlButtonDown()) { // released & previous press recorded
            state_.setControlButtonDown(false);
            // if the other button is down, make sure its subsequent release will not trigger anything, emit long double press if long press, or ignore short press
            if (state_.volumeButtonDown()) {
                state_.setVolumeButtonDown(false); // make sure the other button's release won't trigger anything
                if (longPressCounter_ == 0)
                    state_.setDoubleButtonLongPress();
            // otherwise emit long or short press accordingly
            } else if (longPressCounter_ == 0) {
                state_.setControlButtonLongPress();
            } else {
                state_.setControlButtonPress();
            }
            setIrq();
        }
    }

    static void volumeButtonChanged() {
        volumeBtn_.poll();
        // if we are sleeping check if we should wake up 
        if (status_.sleep) {
            checkButtonWakeup(volumeBtn_.pressed());
        // if this is a press, reset the long press counter and update the state
        } else if (volumeBtn_.pressed()) {
            longPressCounter_ = BUTTON_LONG_PRESS_TICKS;
            state_.setVolumeButtonDown();
            setIrq();
        } else if (state_.volumeButtonDown()) { // released & previous press recorded
            state_.setVolumeButtonDown(false);
            // if the other button is down, make sure its subsequent release will not trigger anything, emit long double press if long press, or ignore short press
            if (state_.controlButtonDown()) {
                state_.setControlButtonDown(false); // make sure the other button's release won't trigger anything
                if (longPressCounter_ == 0)
                    state_.setDoubleButtonLongPress();
            // otherwise emit long or short press accordingly
            } else if (longPressCounter_ == 0) {
                state_.setVolumeButtonLongPress();
            } else {
                state_.setVolumeButtonPress();
            }
            setIrq();
        }
    }

    static void checkButtonWakeup(bool down) {
        if (down) {
            powerOnCountdown_ = POWER_ON_PRESS_TICKS << 16; // from 1/64th of a second to 1/1024th
            powerOnRTCValue_ = RTC.CNT; 
        } else if (powerOnCountdown_ == 0) {
            status_.sleep = false;
        }
    }
    //@}

    /** \name Knob value changes [ISR]
     
        Corresponds to the the change of rotation encoders' position. Runs in ISR.
     */
    //@{
    static void controlKnobChanged() {
        control_.poll();
        state_.setControlValue(control_.value());
        setIrq();
    }

    static void volumeKnobChanged() {
        volume_.poll();
        state_.setVolumeValue(volume_.value());
        setIrq();
    }
    //@}

    inline static RotaryEncoder control_{CTRL_A, CTRL_B, 256};
    inline static RotaryEncoder volume_{VOL_A, VOL_B, 16};
    inline static Button controlBtn_{CTRL_BTN};
    inline static Button volumeBtn_{VOL_BTN};

//@}

/** \name RGB LED Light Strip
 
    The strip displays the following information:

    Special Effects

    Night Light Effects

    These are the default lights. They occupy the entire strip if no notifications are currently active, or are reduced to the inner 6 LEDs if there are any notifications available. 

    Notifications 

    Notifications are displayed only over night lights, or when idle. They take over the leftmost and rightmost LEDs for their own purposes and display as short pulses of various colors. The following notifications are supported:


 */
//@{

    static Color getModeColor(Mode mode) {
        switch (mode) {
            case Mode::MP3:
                return MODE_COLOR_MP3;
            case Mode::Radio:
                return MODE_COLOR_RADIO;
            case Mode::WalkieTalkie:
                return MODE_COLOR_WALKIE_TALKIE;
            case Mode::NightLight:
                return MODE_COLOR_NIGHT_LIGHT;
            default:
                return DEFAULT_COLOR;
        }
    }

    /** A tick of the lights system. 
     
        Called by the main thread every 1/64th of a second. Updates the LED strip according to the current state. Note that if 
     */
    static void lightsTick() {
        uint8_t step = 1;
        if (state_.controlButtonDown() && longPressCounter_ < BUTTON_LONG_PRESS_THRESHOLD) {
            // if the long press counter is not 0, display the countdown bar
            if (longPressCounter_ != 0) {
                Color color = state_.volumeButtonDown() ? DOUBLE_LONG_PRESS_COLOR : getModeColor(state_.mode());
                color = color.withBrightness(ex_.settings.maxBrightness);
                strip_.showBar(BUTTON_LONG_PRESS_TICKS - longPressCounter_, BUTTON_LONG_PRESS_TICKS, color);
                step = 255;
            } else {
                Color color = state_.volumeButtonDown() ? DOUBLE_LONG_PRESS_COLOR : getModeColor(ex_.getNextMode(state_.mode()));
                color = color.withBrightness(ex_.settings.maxBrightness);
                strip_.fill(color);
            }
            // set effect timeout to one, which will immediately trigger revert back to night lights mode as soon as the button is released
            effectTimeout_ = 1;
        } else if (effectTimeout_ > 0) {
            // we don't really have to do anything here when special effect is playing as the strip contains already the required values. Just count down to return back to the night lights mode
            if (--effectTimeout_ == 0) {
                effectHue_ = ex_.nightLight.colorHue();
                effectColor_ = Color::HSV(effectHue_, 255, ex_.settings.maxBrightness);
                if (ex_.nightLight.effect == NightLightEffect::AudioLights) {
                    effect_.audio.maxDelta = 0;
                    effect_.audio.minDelta = 255;
                }
            }
        } else {
            nightLightsTick(step);
        }
        // once we have the tick, update the actual neopixels with the calculated strip value & step
        neopixels_.moveTowardsReversed(strip_, step);
        neopixels_.update();
    }

    /** Updates the neopoixels to show the selected night light effect. 
     
        
     */
    static void nightLightsTick(uint8_t & step) {
        // update the hue of the effect color, if in rainbow mode
        if (/*tickCountdown_ % 16 == 0 && */ ex_.nightLight.hue == NightLightState::HUE_RAINBOW) {
            effectHue_ += 1;
            effectColor_ = Color::HSV(effectHue_, 255, ex_.settings.maxBrightness);
        }
        switch (ex_.nightLight.effect) {
            // turn off the strip, don't change step so that the fade to black is gradual...
            case NightLightEffect::Off:
            default:
                strip_.fill(Color::Black());
                return;
            // 
            case NightLightEffect::AudioLights: {
                if (state_.idle()) {
                    strip_.fill(Color::Black());
                } else {
                    uint8_t ri = recordingWrite_;
                    uint8_t min = 255;
                    uint8_t max = 0;
                    for (uint8_t i = 0; i < 125; ++i) {
                        uint8_t x = recordingBuffer_[--ri];
                        min = (x < min) ? x : min;
                        max = (x > max) ? x : max;
                    }
                    uint8_t d = max - min;
                    effect_.audio.maxDelta = (d > effect_.audio.maxDelta) ? d : effect_.audio.maxDelta;
                    effect_.audio.minDelta = (d < effect_.audio.minDelta) ? d : effect_.audio.minDelta;
                    strip_.showBarCentered(
                        d - effect_.audio.minDelta,
                        max(effect_.audio.maxDelta - effect_.audio.minDelta, 2),
                        effectColor_
                    );
                    // fade the delta range at 1/4 the speed
                    if (tickCountdown_ % 4 == 0) {
                        if (effect_.audio.maxDelta > d)
                            --effect_.audio.maxDelta;
                        if (effect_.audio.minDelta < d)
                            ++effect_.audio.minDelta;
                    }
                }
                step = 255;
                return;
            }
            case NightLightEffect::Breathe: {
                strip_.fill(effectColor_.withBrightness(effectCounter_ & 0xff));
                break;
            }
            case NightLightEffect::BreatheBar: {
                //strip_.centeredBar(effectColor_, effectCounter_ & 0xff, 255);
                strip_.showBarCentered((effectCounter_ & 0xff) / 4, 64, effectColor_);
                break;
            }
            case NightLightEffect::KnightRider: {
                strip_.showPoint((effectCounter_ & 0xff) / 4, 64, effectColor_);
                break;
            }
            case NightLightEffect::StarryNight: {
                strip_.fill(Color::Black());
                return;
            }
            // solid color that simply fills the whole strip with the effect color
            case NightLightEffect::SolidColor: {
                strip_.fill(effectColor_);
                return;
            }
        }
        if ((effectCounter_ & 0xff00) == 0) {
            if (++effectCounter_ == 0x100)
                effectCounter_ = 0x1ff;
        } else {
            if (--effectCounter_ == 0xff)
                effectCounter_ = 0;
        }
        step = 255;
    }

    inline static NeopixelStrip<NEOPIXEL, 8> neopixels_;
    inline static ColorStrip<8> strip_;
    inline static Color effectColor_;
    inline static uint16_t effectHue_;
    inline static uint16_t effectCounter_;

    /** Union of effect settings. 
     
        These are both effects that come from the requests (point, bar, centered bar, etc.) and settings for the night light effects. They can share memory as the night-light effects can always be restarted from their initial settings after the special effect ends. 
     */
    inline static union {
        struct {
            Color timer;
            Color success;
        } longPress;
        struct {
            uint16_t at;
            uint16_t max;
        } point;
        struct {
            uint16_t size;
            uint16_t max;
        } bar;
        struct {
            uint8_t maxDelta;
            uint8_t minDelta;
        } audio;
        struct {
            uint16_t size;
        } breathe;
        struct {
            uint16_t pos;
        } knightRider;

    } effect_;

    /** Timeout in ticks for the requested effect to remain visible. 
     
        After the timeout reaches 0, the night-light mode will be reinstated. 
     */
    inline static uint8_t effectTimeout_;
//@}

/** \name Communication with ESP8266
 
    Communication with the ESP is done via I2C, where ATTiny is slave and ESP is master. ESP can send commands and read status & information from the ATTiny. 

    To send a command the following must be sent, whereas the command id + payload must not exceed 32 bytes. Only one command at a time can be sent and no further commands can be sent before the command is processed (this includes explicitly offset reads - see below):
    
    START, address+W, the command id and optional payload, STOP. 
    
    Reading from the ATTiny retuns by default the state, or if recording audio the audio buffer followed by the State. However, at any time, the offset from which data will be sent can be specified explicitly via the repeated start condition, i.e.: 

    START, address+W, offset, START, address+R, read the bytes, STOP

    Setting the offset only affects the repeated start read. Setting the offset to 0 will always return state. Starting the transaction without the offset write does the default action (state / recorded audio). 

    When there is any change in the peripherals controlled by the ATTiny, it signals the ESP by pulling the AVR_IRQ pin low. Once set, the ESP must initiate slave write operation within the specified ammount of ticks, or AVR will perform a hard reset.   
 */
//@{

    static void initializeI2C() {
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
    
    static void setIrq() {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (! status_.irq) {
                status_.irq = true;
                irqCountdown_ = IRQ_RESPONSE_TIMEOUT;
                pinMode(AVR_IRQ, OUTPUT);
                digitalWrite(AVR_IRQ, LOW);
            }
        }
    }

    static void clearIrq() {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (status_.irq) {
                pinMode(AVR_IRQ, INPUT);
                status_.irq = false;
                // let the countdown to reach 0 naturally
            }
        }
    }

    /** Hard-resets the ESP8266. 
     */
    static void resetESP() {
        clearIrq(); // this is necessary so that ESP boots into a normal mode

    }

    static void processCommand() {
        // TODO process the I2C commands from ESP
        switch (i2cRxBuffer_[0]) {
            /* Sets the specified part of the extended state to the given values via a simple memcpy.
             */
            case msg::SetExtendedState::Id: {
                auto m = pointer_cast<msg::SetExtendedState*>(& i2cRxBuffer_);
                uint8_t * target = pointer_cast<uint8_t*>(&ex_) + m->offset;
                memcpy(target, i2cRxBuffer_ + sizeof(msg::SetExtendedState), m->size);
                break;
            }
            case msg::PowerOff::Id: {
                // TODO
                break;
            }
            case msg::SetMode::Id: {
                auto m = pointer_cast<msg::SetMode*>(& i2cRxBuffer_);
                state_.setMode(m->mode);
                if (m->mode == Mode::Radio)
                    digitalWrite(AUDIO_SRC, AUDIO_SRC_RADIO);
                else 
                    digitalWrite(AUDIO_SRC, AUDIO_SRC_ESP);
                break;
            }
            case msg::SetIdle::Id: {
                auto m = pointer_cast<msg::SetIdle*>(& i2cRxBuffer_);
                state_.setIdle(m->idle);
                if (state_.idle())
                    stopAudioCapture();
                else
                    startAudioCapture(AudioADCSource::Audio);
                break;
            }
            case msg::SetControlRange::Id: {
                auto m = pointer_cast<msg::SetControlRange*>(& i2cRxBuffer_);
                control_.setValues(m->value, m->max);
                state_.setControlValue(m->value);
                break;
            }
            case msg::SetVolumeRange::Id: {
                auto m = pointer_cast<msg::SetVolumeRange*>(& i2cRxBuffer_);
                volume_.setValues(m->value, m->max);
                state_.setVolumeValue(m->value);
                break;
            }
            case msg::LightsFill::Id: {
                auto m = pointer_cast<msg::LightsFill*>(& i2cRxBuffer_);
                strip_.fill(m->color);
                effectTimeout_ = m->timeout;
                break;
            }
            case msg::LightsPoint::Id: {
                auto m = pointer_cast<msg::LightsPoint*>(& i2cRxBuffer_);
                strip_.showPoint(m->value, m->max, m->color);
                effectTimeout_ = m->timeout;
                break;
            }
            case msg::LightsBar::Id: {
                auto m = pointer_cast<msg::LightsPoint*>(& i2cRxBuffer_);
                strip_.showBar(m->value, m->max, m->color);
                effectTimeout_ = m->timeout;
                break;
            }
            case msg::LightsBarCentered::Id: {
                auto m = pointer_cast<msg::LightsPoint*>(& i2cRxBuffer_);
                strip_.showBarCentered(m->value, m->max, m->color);
                effectTimeout_ = m->timeout;
                break;
            }
            case msg::LightsColors::Id: {
                auto m = pointer_cast<msg::LightsColors*>(& i2cRxBuffer_);
                for (uint8_t i = 0; i < 8; ++i)
                    strip_[i] = m->colors[i];
                effectTimeout_ = m->timeout;
                break;
            }
            case msg::StartRecording::Id:
            case msg::StopRecording::Id:
            default:
                // TODO what to do in such an error? 
                break;

        }

        // clear the ready flag and reset the buffer offset - since the bitfield is shared, we must protect its writes
        cli();
        status_.i2cRxReady = false;
        sei();
        i2cRxOffset_ = 0;
    }

    friend void ::TWI0_TWIS_vect();

    /** Number of bytes to be sent from the I2C TX buffer. 
     */
    inline static uint8_t i2cTxLength_ = 0;
    inline static uint8_t * i2cTxBuffer_ = nullptr;
    inline static uint8_t i2cRxBuffer_[32];
    inline static uint8_t i2cRxOffset_ = 0;

//@}

/** \name I2C Transmissible data
 
    */
//@{

    inline static constexpr uint8_t I2C_TRANSMISSIBLE_DATA_SIZE =
        sizeof(State) +
        sizeof(ExtendedState) + 
        sizeof(DateTime) +
        sizeof(DateTime);
    inline static volatile State state_;
    // no need for this to be volatile as the updates and reads are non-confliciting
    inline static ExtendedState ex_;
//@}

/** \name Audio Recording information & buffer. 
 
    */
//@{ 

    static_assert(AUDIO_ADC == 12, "Must be PC2, ADC1 input 8");
    static_assert(MIC == 10, "Must be PC0, ADC1 input 6");

    enum class AudioADCSource : uint8_t {
        Audio = ADC_MUXPOS_AIN8_gc,
        Mic = ADC_MUXPOS_AIN6_gc
    }; // AudioADCSource

    static void initializeAudioCapture() {
        // configure the timer we use for 8kHz audio sampling
        TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc;
        TCB0.CTRLB = TCB_CNTMODE_INT_gc;
        TCB0.CCMP = 1000; // for 8kHz
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
        VREF.CTRLC &= ~ VREF_ADC1REFSEL_gm;
        VREF.CTRLC |= VREF_ADC1REFSEL_1V1_gc;
        // configure the event system - ADC1 to sample when triggered by TCB0
        EVSYS.SYNCCH0 = EVSYS_SYNCCH0_TCB0_gc;
        EVSYS.ASYNCUSER12 = EVSYS_ASYNCUSER12_SYNCCH0_gc;
        // set ADC1 settings (mic & audio) - clkdiv by 2, internal voltage reference
        ADC1.CTRLB = ADC_SAMPNUM_ACC8_gc;
        ADC1.CTRLC = ADC_PRESC_DIV2_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm; // 2mhz
        ADC1.EVCTRL = ADC_STARTEI_bm;
    }

    static void startAudioCapture(AudioADCSource source) {
        recordingRead_ = 0;
        recordingWrite_ = 0;
        // select ADC channel to either MIC or audio ADC
        ADC1.MUXPOS  = static_cast<uint8_t>(source);
        // enable and use 8bit resolution, freerun mode
        ADC1.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_8BIT_gc /*| ADC_FREERUN_bm */;
        // enable the interrupt
        ADC1.INTCTRL |= ADC_RESRDY_bm;
        // start the timer
        TCB0.CTRLA |= TCB_ENABLE_bm;
    }

    static void stopAudioCapture() {
        ADC1.CTRLA = 0;
        TCB0.CTRLA &= ~TCB_ENABLE_bm;
    }

    friend void ::ADC1_RESRDY_vect();

    inline static volatile uint8_t recordingWrite_ = 0;
    inline static volatile uint8_t recordingRead_ = 0;
    inline static volatile uint8_t recordingBuffer_[256];
//@}

/** \name Diagnostic functions
 */
//@{

    /** Shows the given byte displayed on the neopixels strip and freezes. 
     
        The byte is displayed LSB first when looking from the front, or MSB first when looking from the back. 
     */
    static void showByte(uint8_t value, Color const & color) {
        cli();
        neopixels_[7] = (value & 1) ? color : Color::Black();
        neopixels_[6] = (value & 2) ? color : Color::Black();
        neopixels_[5] = (value & 4) ? color : Color::Black();
        neopixels_[4] = (value & 8) ? color : Color::Black();
        neopixels_[3] = (value & 16) ? color : Color::Black();
        neopixels_[2] = (value & 32) ? color : Color::Black();
        neopixels_[1] = (value & 64) ? color : Color::Black();
        neopixels_[0] = (value & 128) ? color : Color::Black();
        neopixels_.update();
        while (true) { };
    }
//@}

    inline static volatile struct {
        /** If true, AVR should sleep, or is currently sleeping.
         */
        bool sleep : 1;
        /** If true, the IRQ flag is currently raised, which is a notification to ESP that it should read the state and react to the changes (or recording buffer if we are recording).
         */        
        bool irq : 1;

        /** If true, there has been 1/64th of a second tick. 
         */
        bool tick: 1;

        /** If true, there is a command received from ESP in the I2C tx buffer that should be processed. 
         */
        bool i2cRxReady : 1;

        /** If true, audio recording is in progress. Transmit requests will transmit the recording buffer by default and having new 32 recorded values triggers the irq. 
         */
        bool recording : 1;

        bool longPressPending : 1;


    } status_;

    static inline uint8_t irqCountdown_;
    static inline uint8_t tickCountdown_; 
    static inline uint16_t powerOnCountdown_ = 0;
    static inline uint16_t powerOnRTCValue_ = 0;
    static inline uint8_t longPressCounter_ = 0; // [isr]



} __attribute__((packed)); // Player


#define I2C_DATA_MASK (TWI_DIF_bm | TWI_DIR_bm) 
#define I2C_DATA_TX (TWI_DIF_bm | TWI_DIR_bm)
#define I2C_DATA_RX (TWI_DIF_bm)
#define I2C_START_MASK (TWI_APIF_bm | TWI_AP_bm | TWI_DIR_bm)
#define I2C_START_TX (TWI_APIF_bm | TWI_AP_bm | TWI_DIR_bm)
#define I2C_START_RX (TWI_APIF_bm | TWI_AP_bm)
#define I2C_STOP_MASK (TWI_APIF_bm | TWI_DIR_bm)
#define I2C_STOP_TX (TWI_APIF_bm | TWI_DIR_bm)
#define I2C_STOP_RX (TWI_APIF_bm)

/** RTC Timer Tick [ISR]


 */
ISR(RTC_PIT_vect) {
    //digitalWrite(AUDIO_SRC, HIGH);
    RTC.PITINTFLAGS = RTC_PI_bm;
    if (Player::status_.sleep) {
        uint16_t x = 1024 - Player::powerOnRTCValue_;
        Player::powerOnCountdown_ = (x > Player::powerOnCountdown_) ? 0 : (Player::powerOnCountdown_ - x);
        Player::powerOnRTCValue_ = 0;
        // do one second tick
        Player::ex_.time.secondTick();

        // TODO check alarm, set wakeup bits and wake up? 
    } else {
        // set the tick flag so that more resource demanding periodic tasks can be done in the loop
        Player::status_.tick = true;
        // decrement irq
        if (Player::irqCountdown_ > 0)
            --Player::irqCountdown_;
        // decrement button long press counter
        if (Player::longPressCounter_ > 0)
            --Player::longPressCounter_;
    }
    //digitalWrite(AUDIO_SRC, LOW);
}

/** [ISR]
 */
ISR(TWI0_TWIS_vect) {
    //digitalWrite(AUDIO_SRC, HIGH);
    uint8_t status = TWI0.SSTATUS;
    // sending data to accepting master is on our fastpath as is checked first
    if ((status & I2C_DATA_MASK) == I2C_DATA_TX) {
        // if there is a byte to be sent as part of current transaction, do that and ack
        if (Player::i2cTxLength_ > 0) {
            --Player::i2cTxLength_;
            TWI0.SDATA = *(Player::i2cTxBuffer_++);
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
            // if we have just sent the events part of the state, clear them so that new events can be written - as we do this inside an IRQ that just copied the particular byte, this is the best way of making sure new events can be added after these were sent
            // of course, if the I2C transmission would fail, then we have lost the events, but if the transmission fails, the software has bigger problems to think about
            if (Player::i2cTxBuffer_ == (uint8_t *)(& Player::state_) + 2)
                Player::state_.clearEvents();
        // after the transaction has been trasnmitted, switch to sending the state, if not already sending it
        } else if (Player::i2cTxBuffer_ != (uint8_t*)(& Player::state_ + 1)) {
            // if we have successfully sent a recording buffer, increment the read index accordingly
            if (Player::i2cTxBuffer_ > Player::recordingBuffer_)
                Player::recordingRead_ += 32;
            // move the tx buffer to state and start sending
            Player::i2cTxBuffer_ = (uint8_t*)(& Player::state_);
            Player::i2cTxLength_ = sizeof(State);
            TWI0.SDATA = *(Player::i2cTxBuffer_++);
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
        // otherwise there is nothing more to send, nack
        } else {
            TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
        }
    // a byte has been received from master. Store it and send either ACK if we can store more, or NACK if we can't store more
    } else if ((status & I2C_DATA_MASK) == I2C_DATA_RX) {
        Player::i2cRxBuffer_[Player::i2cRxOffset_++] = TWI0.SDATA;
        TWI0.SCTRLB = (Player::i2cRxOffset_ == 32) ? TWI_SCMD_COMPTRANS_gc : TWI_SCMD_RESPONSE_gc;
    // master requests slave to write data, determine what data to send, send ACK if there is data to be transmitted, NACK if there is no data to send
    } else if ((status & I2C_START_MASK) == I2C_START_TX) {
        Player::clearIrq();
        // if i2cready is not set, and rx offset is 1, we have previously received one byte, but STOP condition has not been sent, i.e. this is repeated start, and the received byte is TxOffset
        if (Player::i2cRxOffset_ == 1 && ! Player::status_.i2cRxReady) {
            Player::i2cTxBuffer_ = (uint8_t*)(& Player::state_) + Player::i2cRxBuffer_[0];
            Player::i2cTxLength_ = Player::I2C_TRANSMISSIBLE_DATA_SIZE - Player::i2cRxBuffer_[0];
            Player::i2cRxOffset_ = 0;
        // otherwise, if currently recording, prepare to send next 32 bytes of audio, if available        
        } else if (Player::status_.recording) {
            Player::i2cTxBuffer_ = (uint8_t *)(& Player::recordingBuffer_) + Player::recordingRead_;
            Player::i2cTxLength_ = (((Player::recordingRead_ + 32) & 0xff) <= Player::recordingWrite_) ? 32 : 0;
        // or if not recording, do the default action, which is to send the state
        } else {
            Player::i2cTxBuffer_ = (uint8_t*)(& Player::state_);
            Player::i2cTxLength_ = Player::I2C_TRANSMISSIBLE_DATA_SIZE;
        }
        // depending on the available length of data to be sent either ACK or NACK if there is nothing to send
        if (Player::i2cTxLength_ > 0) 
            TWI0.SCTRLB = TWI_ACKACT_ACK_gc + TWI_SCMD_RESPONSE_gc;
        else 
            TWI0.SCTRLB = TWI_ACKACT_NACK_gc + TWI_SCMD_COMPTRANS_gc;
    // master requests to write data itself. ACK if the buffer is empty (we do not support multiple commands in same buffer), NACK otherwise.
    } else if ((status & I2C_START_MASK) == I2C_START_RX) {
        if (Player::i2cRxOffset_ == 0)
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
        else
            TWI0.SCTRLB = TWI_ACKACT_NACK_gc;
    // when a transmission finishes we must see if another transmission required and re-raise the irq flag for ESP. While recording this means we need to see if there is another 32 bytes available yet
    } else if ((status & I2C_STOP_MASK) == I2C_STOP_TX) {
        if (Player::status_.recording) {
            // if the read and write offsets are into the same 32 bit partition, then the flag will be raised when more samples are available, otherwise raise the flag now
            if (Player::recordingRead_ / 32 != Player::recordingWrite_ / 32)
                Player::setIrq();
        }
        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
    } else if ((status & I2C_STOP_MASK) == I2C_STOP_RX) {
        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
        Player::status_.i2cRxReady = true;
    } else {
        Player::showByte(status, Color::Red());
    }
    //digitalWrite(AUDIO_SRC, LOW);
}

ISR(ADC1_RESRDY_vect) {
    //digitalWrite(AUDIO_SRC, HIGH);
    Player::recordingBuffer_[Player::recordingWrite_++] = (ADC1.RES / 8) & 0xff;
    if (Player::recordingWrite_ % 32 == 0 && Player::status_.recording)
        Player::setIrq();
    //digitalWrite(AUDIO_SRC, LOW);
}

void setup() {
    Player::initialize();
}

void loop() {
    Player::loop();
}



#endif

#ifdef HAHA



/** ATTiny part of the player.     
    
 */
class Player {
public:
    Player() = delete;

    /** Initializes the chip. 
     */
    static void Initialize() {
        // disable the peripheral clock divider
        //CPU_CCP = CCP_IOREG_gc;
        //CLKCTRL.MCLKCTRLB &= ~ CLKCTRL_PEN_bm; 
        // configure the timer we use for 8kHz audio sampling
        TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc;
        TCB0.CTRLB = TCB_CNTMODE_INT_gc;
        TCB0.CCMP = 1000; // for 8kHz
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
        // set ADC1 settings (mic & audio) - clkdiv by 2, internal voltage reference
        ADC1.CTRLB = ADC_SAMPNUM_ACC8_gc;
        ADC1.CTRLC = ADC_PRESC_DIV2_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm; // 2mhz
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
        /** Indicates that the latest long press of either volume or control button happened when the other one was down and therefore can lead to a double long press. 
         */
        bool doubleLongPress : 1; 
        bool recording : 1; // indicates that we are recording 
    } Status_;

    inline static State State_;

    inline static DateTime Time_;

/** \name Comms
 */
//@{
private:


    static void I2CReceive() {
        // TODO check that numBytes <= message length or accept corrupted messages?
        switch (I2C_RX_Buffer_[0]) {
            case msg::PowerOff::Id: {
                // set sleep to true so that the sleep can be handled in main loop
                Status_.sleep = true;
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
            case msg::SetAudioLights::Id: {
                auto msg = msg::At<msg::SetAudioLights>(I2C_RX_Buffer_);
                msg.applyTo(State_);
                if (State_.audioLights()) 
                    StartAudioADC(AudioADCSource::Audio);
                else
                    StopAudioADC();
                break;
            }
            case msg::Play::Id: {
                // TODO enable sound out
                if (State_.audioLights()) 
                    StartAudioADC(AudioADCSource::Audio);
                break;
            }
            case msg::Pause::Id: {
                // TODO disable sound out
                StopAudioADC();
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

//@}



/** \name Knobs & Buttons
 
    
 */
//@{

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
            // if the counter is greater than 0, we are dealing with a short press
            if (ControlBtnCounter_ > 0) {
                if (Status_.doubleLongPress == true)
                    State_.setVolumeLongPress();
                State_.setControlPress();
                ControlBtnCounter_ = 0;
                Status_.doubleLongPress = false;
            // if the counter is 0, we are looking at a long press
            } else {
                if (Status_.doubleLongPress == true) { // the other button has been long pressed
                    State_.setDoubleLongPress();
                    Status_.doubleLongPress = false;
                } else if (State_.volumeDown()) { // this long press has to be delayed as it might be double
                    Status_.doubleLongPress = true;
                } else { // emit long press
                    State_.setControlLongPress();
                    // wake up from sleep if sleeping
                    Status_.sleep = false;
                }
            }
        }
        // if we are not sleeping, set the IRQ to notify ESP. If we have just woken up, then this is irrelevant and will be cleared when waking ESP up
        // when recording, don't notify either as it is always transmitted in the first state byte
        if (! Status_.sleep && ! Status_.recording)
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
            // if the counter is greater than 0, we are dealing with a short press
            if (VolumeBtnCounter_ > 0) {
                if (Status_.doubleLongPress == true)
                    State_.setControlLongPress();
                State_.setVolumePress();
                VolumeBtnCounter_ = 0;
                Status_.doubleLongPress = false;
            // if the counter is 0, we are looking at a long press
            } else {
                if (Status_.doubleLongPress == true) { // the other button has been long pressed
                    State_.setDoubleLongPress();
                    Status_.doubleLongPress = false;
                } else if (State_.controlDown()) { // this long press has to be delayed as it might be double
                    Status_.doubleLongPress = true;
                } else { // emit long press
                    State_.setVolumeLongPress();
                    // wake up from sleep if sleeping
                    Status_.sleep = false;
                }
            }
        }
        // if we are not sleeping, set the IRQ to notify ESP. If we have just woken up, then this is irrelevant and will be cleared when waking ESP up
        // when recording, don't notify either as it is always transmitted in the first state byte
        if (! Status_.sleep && !Status_.recording)
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
        // when a button is down then the strip shows the long press progress first and foremost, unless we are recoding in which case display the audiolights of the recorded message
        if (State_.volumeDown() || State_.controlDown()) {
            if (Status_.recording) {
                AudioLights(step);
            } else {
                uint8_t v = BUTTON_LONG_PRESS_TICKS - max(VolumeBtnCounter_, ControlBtnCounter_);
                if (v == BUTTON_LONG_PRESS_TICKS)
                    Lights_.fill(BUTTONS_LONG_PRESS_COLOR.withBrightness(MaxBrightness_));
                else if (v >= BUTTON_LONG_PRESS_DELAY)
                    Lights_.showBar(v - BUTTON_LONG_PRESS_DELAY, BUTTON_LONG_PRESS_TICKS - BUTTON_LONG_PRESS_DELAY, BUTTONS_LONG_PRESS_COLOR.withBrightness(MaxBrightness_));
            }
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
        } else if (State_.audioLights()) {
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
        uint8_t ri = RecordingWrite_;
        uint8_t audioMin = 255;
        uint8_t audioMax = 0;
        for (uint8_t i = 0; i < 125; ++i) {
            uint8_t x = RecordingBuffer_[--ri];
            audioMin = (x < audioMin) ? x : audioMin;
            audioMax = (x > audioMax) ? x : audioMax;
        }
        uint8_t v = audioMax - audioMin;
        if (v > AudioLightsMax_)
            AudioLightsMax_ = v;
        if (v < AudioLightsMin_)
            AudioLightsMin_ = v;
        Lights_.showCenteredBar(v - AudioLightsMin_, AudioLightsMax_ - AudioLightsMin_, AccentColor_.withBrightness(MaxBrightness_));
        if (Status_.ticksCounter % 4 == 0) {
            if (AudioLightsMax_ > v)
                --AudioLightsMax_;
            if (AudioLightsMin_ < v)
                ++AudioLightsMin_;
        }
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
    inline static volatile uint8_t AudioLightsMin_;

//@}


/** \name Audio & Microphone Listening
 */
//@{
private:

    enum class AudioADCSource : uint8_t {
        Audio = ADC_MUXPOS_AIN8_gc,
        Mic = ADC_MUXPOS_AIN6_gc
    }; // AudioADCSource


    static void StartAudioADC(AudioADCSource channel) {
        AudioLightsMin_ = 255;
        AudioLightsMax_ = 0;
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
    //digitalWrite(AUDIO_SRC, HIGH);
    RTC.PITINTFLAGS = RTC_PI_bm;
    Player::Tick();
    //digitalWrite(AUDIO_SRC, LOW);
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
            TWI0.SDATA = Player::I2C_TX_Buffer_[Player::I2C_TX_Offset_++];
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
            // first byte of state is the events of the buttons, so once we send them, clear them so that we don't send them again
            if (Player::I2C_TX_Offset_ == 1 && Player::I2C_TX_Buffer_ == pointer_cast<uint8_t*>(& Player::State_))
                Player::State_.clearButtonEvents();
        // after the selected buffer has been sent, if the buffer is was not state, switch to state and continue sending as long as master wants more data
        } else if (Player::I2C_Current_TX_Mode_ != Player::I2C_TX_Mode::State) {
            if (Player::I2C_Current_TX_Mode_ == Player::I2C_TX_Mode::Recording)
                Player::RecordingRead_ += 32; // we succeeded in sending the buffer, increment read index
            Player::I2C_Current_TX_Mode_ = Player::I2C_TX_Mode::State;
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
        Player::I2C_Current_TX_Mode_ = Player::I2C_TX_Mode_;
        switch (Player::I2C_Current_TX_Mode_) {
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
        if (Player::I2C_TX_Offset_ < Player::I2C_TX_Length_) 
            TWI0.SCTRLB = TWI_ACKACT_ACK_gc + TWI_SCMD_RESPONSE_gc;
        else 
            TWI0.SCTRLB = TWI_ACKACT_NACK_gc + TWI_SCMD_COMPTRANS_gc;
    // master requests to write data itself. ACK if the buffer is empty (we do not support multiple commands in same buffer), NACK otherwise.
    } else if ((status & I2C_START_MASK) == I2C_START_RX) {
        if (Player::I2C_RX_Offset_ == 0)
            TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
        else
            TWI0.SCTRLB = TWI_ACKACT_NACK_gc;
    // when a transmission finishes we must see if another transmission required and re-raise the irq flag for ESP. While recording this means we need to see if there is another 32 bytes available yet
    } else if ((status & I2C_STOP_MASK) == I2C_STOP_TX) {
        if (Player::Status_.recording) {
            // if the read and write offsets are into the same 32 bit partition, then the flag will be raised when more samples are available, otherwise raise the flag now
            if (Player::RecordingRead_ / 32 != Player::RecordingWrite_ / 32)
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
    Player::RecordingBuffer_[Player::RecordingWrite_++] = (ADC1.RES / 8) & 0xff;
    if (Player::RecordingWrite_ % 32 == 0 && Player::Status_.recording)
        Player::SetIrq();
    //digitalWrite(AUDIO_SRC, LOW);
}




#endif // HAHA

#endif // ARCH_ATTINY
