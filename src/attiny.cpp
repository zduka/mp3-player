#if (defined ARCH_ATTINY)

#if (F_CPU!=8000000UL)
//#error "Only 8Mhz clock is supported"
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
#define DEBUG_PIN 7
#define CHARGING 5


#define AUDIO_SRC_ESP HIGH
#define AUDIO_SRC_RADIO LOW

#define DCDC_PWR_ON LOW
#define DCDC_PWR_OFF HIGH


extern "C" void RTC_PIT_vect(void) __attribute__((signal));
extern "C" void TWI0_TWIS_vect(void) __attribute__((signal));
extern "C" void ADC1_RESRDY_vect(void) __attribute__((signal));

/** We need the actual transmissible state to be in a own structure, guaranteeing that state and extended state are right next to each other in memory.
 */
class AVRState {
public:
    volatile State state;
    ExtendedState ex;
} __attribute__((packed));

static_assert(sizeof(AVRState) == sizeof(State) + sizeof(ExtendedState));

#if (defined TEST_NEOPIXEL)

/** Displays various colors on the neopixel strip after turning on 3v3. 
 */
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

/** Starts the radio at lowest volume and preset station while displaying a moving light on the neopixels. 
 */

RDA5807M radio;
NeopixelStrip<NEOPIXEL, 8> neopixels;
uint8_t x = 0;

void setup() {
    pinMode(DCDC_PWR, OUTPUT);
    digitalWrite(DCDC_PWR, DCDC_PWR_ON);
    pinMode(AUDIO_SRC, OUTPUT);
    digitalWrite(AUDIO_SRC, AUDIO_SRC_RADIO);
    //pinMode(HEADPHONES, OUTPUT);
    //digitalWrite(HEADPHONES, LOW);
    delay(50);
    Wire.begin();
    radio.init();
    delay(50);
    radio.setMono(true);
    delay(50);
    radio.setVolume(1);
    delay(50);
    radio.setBandFrequency(RADIO_BAND_FM, 9370);
}
void loop() { 
    neopixels.showPoint(x, 255, Color::White().withBrightness(32));
    neopixels.update();
    delay(30);
    ++x;
    //if (x % 16 == 0)
    //    radio.setVolume(x >> 4);
}

#else 

/** The actual player. 
  */
class Player {
public:
    static void initialize() {
#ifdef DEBUG_LOG
        Serial.begin(115200);
#endif
        pinMode(DEBUG_PIN, OUTPUT);
        digitalWrite(DEBUG_PIN, LOW);
        delay(10);
        USART0.CTRLB &= ~ USART_RXEN_bm;
        LOG("Initializing AVR");
        initializeAtTiny();
        // enable watchdog
        wdt_enable();

        //pinMode(DEBUG_PIN, OUTPUT);
        //digitalWrite(DEBUG_PIN, LOW);
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
        // proceed with wakeup
        wakeup();
    }

    static void loop() {
        if (status_.tick)
            tick();
        if (status_.i2cRxReady)
            processCommand();
        // if the measurement (headphones, voltage, temp, ...) is ready, measure
        if (ADC0.INTFLAGS & ADC_RESRDY_bm) {
            measurementsADCReady();
            if (undervoltageCountdown_ == 0) {
                criticalBatteryWarning();
                sleep();
            }
        }

    }

private:

/** \name ATTiny Management. 
 */
//@{

    /** Initializes the ATTiny chip. 
     
        Checks the reset reason and updates the state accordingly.
     */ 
    static void initializeAtTiny() {
        if (RSTCTRL.RSTFR & RSTCTRL_PORF_bm) 
            LOG("  power-on reset");
        if (RSTCTRL.RSTFR & RSTCTRL_BORF_bm)
            LOG("  brown-out reset");
        if (RSTCTRL.RSTFR & RSTCTRL_WDRF_bm)
            LOG("  watchdog reset");
        if (RSTCTRL.RSTFR & RSTCTRL_SWRF_bm)
            LOG("  software reset");
        // reset the flags
        RSTCTRL.RSTFR = 0;
    }

    /** Enables the WDT with the longest period (8 seconds). 
     
        Note that WDT should be disabled before going to sleep to prevent resets. 
     */
    static void wdt_enable() {
#ifndef DEBUG_DISABLE_WDT
        _PROTECTED_WRITE(WDT.CTRLA,WDT_PERIOD_8KCLK_gc); // no window, 8sec
#endif
    }

    /** Resets the WDT timer. 
     */
    static void wdt_reset() {
        __asm__ __volatile__ ("wdr"::);
    }

    /** Disables the watchdog. 
     
        This *must* be called before going to sleep. 
     */
    static void wdt_disable() {
        _PROTECTED_WRITE(WDT.CTRLA,0);
    }

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
        static_assert(CHARGING == 5, "Charging pin must be PB4, AIN_9 for the charging detection to work");
        PORTB.PIN4CTRL &= ~PORT_ISC_gm;
        PORTB.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;
        PORTB.PIN4CTRL &= ~PORT_PULLUPEN_bm;
    }
    
    static void tick() {
        //digitalWrite(DEBUG_PIN, HIGH); 
        // TODO enable this in production settings
        if (status_.espBusy || status_.recording)
            wdt_reset();
        if (status_.shouldSleep)
            sleep();           
        // it is possible that between the irq check and the countdown check the irq will be cleared, however the second check would then only pass if the irq countdown was at its end and therefore the reset is ok
        if ((status_.irq || status_.espBusy) && (--irqCountdown_ == 0))
             resetESP();
        // make sure we have a light show
        if (state_.state.mode() != Mode::Sync)
            lightsTick();
        cli();
        status_.tick = false;
        sei();
        // if we have accumulated 64 ticks, do a one second tick check
        if ((++tickCountdown_ % 64) == 0) {
            //LOG("time: %u", state_.ex.time.second());
            secondTick();
            // actually do a proper poweroff - telling ESP first, dimming the lights, etc. 
            // this does not happen in sync mode, where the ESP busy timeout is used instead
            if (state_.state.mode() != Mode::Sync && --powerCountdown_ == 0) {
                LOG("ESP power off countdown");
                state_.state.setMode(Mode::ESPOff);
                setIrq();
            }
        }
        //digitalWrite(DEBUG_PIN, LOW);
    }

    static void secondTick() {
        state_.ex.time.secondTick();
        if (state_.ex.time.hour() == syncHour_  && state_.ex.time.minute() == 0 && state_.ex.time.second() == 0) {
            status_.sync = true;
            status_.sleeping = false;
        } else if (state_.ex.alarm == state_.ex.time) {
            state_.state.setMode(Mode::Alarm);
            setIrq();
            status_.sleeping = false;
        }
    }

    /** Puts the AVR to sleep. 
     */
    static void sleep() {
        LOG("entering sleep");
        status_.shouldSleep = false;
        do { // wrapped in a loop to account to allow wakeup to re-enter sleep immediately (such as when low volatge, etc.)
            // disable rotary encoder interrupts so that they do not wake us from sleep
            control_.clearInterrupt();
            volume_.clearInterrupt();
            // turn off neopixels and esp
            peripheralPowerOff();
            // disable idle mode when sleeping, set mode to music, which is the default after waking up
            status_.espBusy = false;
            state_.state.setIdle(false);
            state_.state.setMode(Mode::Music); 
            state_.state.setWiFiStatus(WiFiStatus::Off);
            // enable RTC interrupt every second so that the time can be kept
            while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
            RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc + RTC_PITEN_bm;
            // enter the sleep mode. Upon wakeup, go to sleep immediately for as long as the sleep mode is on (we wake up every second to increment the clock and when buttons are pressed as well)
            LOG("sleep");
            status_.sleeping = true;
            // clear the reset flags in CPU
            RSTCTRL.RSTFR = 0;
            wdt_disable();
            while (status_.sleeping)
                sleep_cpu();
            wdt_enable();
            // clear the button events that led to the wakeup
            // no need to disable interruts as esp is not running and so I2C can't be reading state at this point
            state_.state.clearEvents();
            // if we are not longer sleeping, wakeup
            wakeup();
        } while (status_.sleeping);
    }

    static void wakeup() {
        LOG("Wakeup");
        if (status_.sync) {
            LOG("Synchronization");
            status_.sync = false;
            state_.state.setMode(Mode::Sync);
        }
        // enable RTC interrupt every 1/64th of a second
        while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
        RTC.PITCTRLA = RTC_PERIOD_CYC16_gc + RTC_PITEN_bm;
        // end the wakeup state and mark both buttons as not down so that their release will be ignored
        status_.wakeup = false;
        state_.state.setVolumeButtonDown(false);
        state_.state.setControlButtonDown(false);
        // start ADC0 to measure the VCC and wait enough measurements to be done to determine if battery is a problem so that we can overcome any initial surges
        ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc;
        ADC0.COMMAND = ADC_STCONV_bm;
        for (uint16_t i = 0; i < UNDERVOLTAGE_TIMEOUT * 3; ++i) {
            // wait for the voltage measurement to finish
            while (! ADC0.INTFLAGS & ADC_RESRDY_bm) {
            }
            measurementsADCReady();
        }
        if (undervoltageCountdown_ == 0) {
            criticalBatteryWarning();
            status_.sleeping = true;
        } else {
            powerCountdown_ = 60; // make sure we are awake for at least 1 minute, the ESP will set proper timer when it turns on
            // enable interrupts for rotary encoders
            control_.setInterrupt(controlKnobChanged);
            volume_.setInterrupt(volumeKnobChanged);
            // enable power to neopixels, esp8266 and other circuits
            peripheralPowerOn();
            // shows the wakeup progressbar if not in sync mode (sync mode does not update neopixels)
            strip_.showBar(1, 8, DEFAULT_COLOR.withBrightness(maxBrightness_));
            effectTimeout_ = SPECIAL_LIGHTS_TIMEOUT;
        }
    }

    /** Shows a critical battery warning before going back to sleep. 
     
        Powers on peripherals with both SDA and SCL held low, which puts ESP immediately in deep sleep and then flashes the full LED strip red three times. 
     */
    static void criticalBatteryWarning(uint8_t brightness = 255) {
        LOG("Critical battery");
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
        LOG("3V3 PowerOn");
        clearIrq();
        pinMode(DCDC_PWR, OUTPUT);
        digitalWrite(DCDC_PWR, DCDC_PWR_ON);
        // add a delay so that the capacitors can be charged, voltage stabilized and ESP started
        delay(100);
        // clear the LED strip
        neopixels_.fill(Color::Black());
        neopixels_.update();
        // enable the headphones interrupt detection and set the current headphones state
        attachInterrupt(digitalPinToInterrupt(HEADPHONES), Player::headphonesChange, CHANGE);
        cli(); // the headphones change expects to run as an interrupt already
        headphonesChange();
        sei();
    }

    static void peripheralPowerOff() {
        LOG("3V3 PowerOff");
        detachInterrupt(digitalPinToInterrupt(HEADPHONES));
        pinMode(HEADPHONES, INPUT);
        pinMode(DCDC_PWR, INPUT);
    }

    /** Triggered when the headphones pin changes its value. 
     */
    static void headphonesChange() {
        state_.state.setHeadphonesConnected(! digitalRead(HEADPHONES));
        setIrq();
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
                state_.ex.measurements.vcc = value;
                sei();
                // set the undevoltage timeout - don't just sleep yet, for better clarity we initiate the sleep in the main loop
                if (value >= BATTERY_CRITICAL_VCC)
                    undervoltageCountdown_ = UNDERVOLTAGE_TIMEOUT;
                else if (undervoltageCountdown_ > 0)
                    --undervoltageCountdown_; 
                // check the battery mode voltage threshold
                bool batteryMode = state_.ex.measurements.vcc <= USB_VOLTAGE_THRESHOLD;
                if (batteryMode != state_.state.batteryMode()) {
                    state_.state.setBatteryMode(batteryMode);
                    setIrq();
                }
                // switch the ADC to be ready to measure the temperature
                ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;
                ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
                break;
            }
            case ADC_MUXPOS_TEMPSENSE_gc: { // tempreature sensor
                // taken from the ATTiny datasheet example
                int8_t sigrow_offset = SIGROW.TEMPSENSE1; 
                uint8_t sigrow_gain = SIGROW.TEMPSENSE0;
                int32_t temp = value - sigrow_offset; // Result might overflow 16 bit variable (10bit+8bit)
                temp *= sigrow_gain;
                // temp is now in kelvin range, to convert to celsius, remove -273.15 (x256)
                temp -= 69926;
                // and now loose precision to 0.25C
                temp = (temp >>= 6) * 30 / 12;
                cli();
                state_.ex.measurements.temp = static_cast<int16_t>(temp);
                sei();
                ADC0.MUXPOS = ADC_MUXPOS_AIN9_gc;
                ADC0.CTRLC = ADC_PRESC_DIV16_gc | ADC_REFSEL_INTREF_gc | ADC_SAMPCAP_bm; // 0.5mhz
                break;
            }
            case ADC_MUXPOS_AIN9_gc: { // charging voltage 
                bool charging = (!state_.state.batteryMode()) && value < 512;
                if (state_.state.charging() != charging) {
                    cli();
                    state_.state.setCharging(charging);
                    sei();
                    setIrq();
                }
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
        if (state_.state.mode() == Mode::Sync) {
            neopixels_.fill(Color::Red().withBrightness(maxBrightness_));
            neopixels_.update();
        }
        if (status_.espBusy)
            return;
        controlBtn_.poll();
        if (status_.sleeping) {
            checkButtonWakeup(controlBtn_.pressed());
        } else if (controlBtn_.pressed()) {
            longPressCounter_ = BUTTON_LONG_PRESS_TICKS;
            state_.state.setControlButtonDown();
            setIrq();
        } else if (state_.state.controlButtonDown()) { // released & previous press recorded
            state_.state.setControlButtonDown(false);
            // if the other button is down, make sure its subsequent release will not trigger anything, emit long double press if long press, or ignore short press
            if (state_.state.volumeButtonDown()) {
                state_.state.setVolumeButtonDown(false); // make sure the other button's release won't trigger anything
                if (longPressCounter_ == 0)
                    state_.state.setDoubleButtonLongPress();
            // otherwise emit long or short press accordingly
            } else if (longPressCounter_ == 0) {
                state_.state.setControlButtonLongPress();
            } else {
                state_.state.setControlButtonPress();
            }
            setIrq();
        }
    }

    static void volumeButtonChanged() {
        if (state_.state.mode() == Mode::Sync) {
            neopixels_.fill(Color::Red().withBrightness(maxBrightness_));
            neopixels_.update();
        }
        if (status_.espBusy)
            return;
        volumeBtn_.poll();
        // if we are sleeping check if we should wake up 
        if (status_.sleeping) {
            checkButtonWakeup(volumeBtn_.pressed());
        // if this is a press, reset the long press counter and update the state
        } else if (volumeBtn_.pressed()) {
            longPressCounter_ = BUTTON_LONG_PRESS_TICKS;
            state_.state.setVolumeButtonDown();
            setIrq();
        } else if (state_.state.volumeButtonDown()) { // released & previous press recorded
            state_.state.setVolumeButtonDown(false);
            // if the other button is down, make sure its subsequent release will not trigger anything, emit long double press if long press, or ignore short press
            if (state_.state.controlButtonDown()) {
                state_.state.setControlButtonDown(false); // make sure the other button's release won't trigger anything
                if (longPressCounter_ == 0)
                    state_.state.setDoubleButtonLongPress();
            // otherwise emit long or short press accordingly
            } else if (longPressCounter_ == 0) {
                state_.state.setVolumeButtonLongPress();
            } else {
                state_.state.setVolumeButtonPress();
            }
            setIrq();
        }
    }

    static void checkButtonWakeup(bool down) {
        if (down) {
            status_.wakeup = true;
            // change the RTC ticks to 1/64th of a second 
            while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
            RTC.PITCTRLA = RTC_PERIOD_CYC16_gc + RTC_PITEN_bm;
            powerCountdown_ = POWER_ON_PRESS_TICKS; 
        } else {
            status_.wakeup = false;
            // go back to 1 second RTC interval and sleep
            while (RTC.PITSTATUS & RTC_CTRLBUSY_bm) {}
            RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc + RTC_PITEN_bm;
        }
    }
    //@}

    /** \name Knob value changes [ISR]
     
        Corresponds to the the change of rotation encoders' position. Runs in ISR.
     */
    //@{
    static void controlKnobChanged() {
        if (status_.espBusy)
            return;
        control_.poll();
        state_.state.setControlValue(control_.value());
        state_.state.setControlTurn();
        setIrq();
    }

    static void volumeKnobChanged() {
        if (status_.espBusy)
            return;
        volume_.poll();
        state_.state.setVolumeValue(volume_.value());
        state_.state.setVolumeTurn();
        setIrq();
    }
    //@}

    inline static RotaryEncoder control_{CTRL_A, CTRL_B, 256};
    inline static RotaryEncoder volume_{VOL_A, VOL_B, MAX_VOLUME};
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

    static Color getModeColor(Mode mode, MusicMode musicMode) {
        switch (mode) {
            case Mode::Music:
                switch (musicMode) {
                    case MusicMode::MP3:
                        return MODE_COLOR_MP3;
                    case MusicMode::Radio:
                        return MODE_COLOR_RADIO;
                    case MusicMode::Disco:
                        return MODE_COLOR_DISCO;
                }
            case Mode::WalkieTalkie:
                return MODE_COLOR_WALKIE_TALKIE;
            case Mode::Lights:
                return MODE_COLOR_LIGHTS;
            default:
                return DEFAULT_COLOR;
        }
    }

    /** A tick of the lights system. 
     
        Called by the main thread every 1/64th of a second. Updates the LED strip according to the current state. Note that if 
     */
    static void lightsTick() {
        bool displayNotifications = false;
        uint8_t step = 1;
        // deal with recording first as it is the highest priority visualization
        if (status_.recording) {
            audioLightsTick(step);
        // then with buttons being pressed
        } else if ((state_.state.controlButtonDown() || state_.state.volumeButtonDown()) && longPressCounter_ < BUTTON_LONG_PRESS_THRESHOLD) {
            if (longPressCounter_ != 0) { // if the long press is not active, show progress bar in the color of the current mode
                Color color = getModeColor(state_.state.mode(), state_.state.musicMode());
                color = color.withBrightness(maxBrightness_);
                strip_.showBar(BUTTON_LONG_PRESS_TICKS - longPressCounter_, BUTTON_LONG_PRESS_TICKS, color);
                step = 255;
            } else {
                Color color;
                // only control button long press - music mode change
                if (! state_.state.volumeButtonDown())
                    color = getModeColor(Mode::Music, getNextMusicMode(state_.state.mode(), state_.state.musicMode(), status_.radioEnabled, status_.discoEnabled));
                // only volume button pressed (and not recording, which is handled above), toggle the lights settings mode on/off
                else if (! state_.state.controlButtonDown()) // TODO what to with walkie-talkie long press
                    color = state_.state.mode() == Mode::Music ? MODE_COLOR_LIGHTS : getModeColor(Mode::Music, state_.state.musicMode());
                // both buttons pressed, in music mode -> toggle music / walkie-talkie
                else
                    color = state_.state.mode() == Mode::Music ? MODE_COLOR_WALKIE_TALKIE : getModeColor(Mode::Music, state_.state.musicMode());
                color = color.withBrightness(maxBrightness_);
                strip_.fill(color);
            }
            effectTimeout_ = 1;
        // then with special effects being currently in process
        } else if (effectTimeout_ > 0) {
            // we don't really have to do anything here when special effect is playing as the strip contains already the required values. Just count down to return back to the night lights mode
            if (--effectTimeout_ == 0) {
                effectHue_ = state_.ex.lights.colorHue();
                effectColor_ = Color::HSV(effectHue_, 255, maxBrightness_);
                if (state_.ex.lights.effect == LightsEffect::AudioLights) {
                    effect_.audio.maxDelta = 0;
                    effect_.audio.minDelta = 255;
                }
            }
        // and finally do the lights tick
        } else {
            lightsTick(step);
            displayNotifications = true;
        }
        // once we have the tick, update the actual neopixels with the calculated strip value & step
        neopixels_.moveTowardsReversed(strip_, step);
        if (displayNotifications)
            lightsNotifications();
        if (powerCountdown_ < 64)
            neopixels_.withBrightness(powerCountdown_ * 4);
        neopixels_.update();
    }

    /** Updates the neopoixels to show the selected night light effect.
     
        
     */
    static void lightsTick(uint8_t & step) {
        // update the hue of the effect color, if in rainbow mode
        if (/*tickCountdown_ % 16 == 0 && */ state_.ex.lights.hue == LightsState::HUE_RAINBOW) {
            effectHue_ += 1;
            effectColor_ = Color::HSV(effectHue_, 255, maxBrightness_);
        }
        //state_.ex.nightLight.effect = LightsEffect::AudioLights;
        switch (state_.ex.lights.effect) {
            // turn off the strip, don't change step so that the fade to black is gradual...
            case LightsEffect::Off:
            default:
                strip_.fill(Color::Black());
                return;
            // 
            case LightsEffect::AudioLights: {
                audioLightsTick(step);
                return;
            }
            case LightsEffect::BinaryClock: {
                switch (state_.ex.time.second() % (state_.state.batteryMode() ? 3 : 2)) {
                    case 0:
                        showByte(state_.ex.time.hour(), BINARY_CLOCK_HOURS.withBrightness(maxBrightness_));
                        break;
                    case 1:
                        showByte(state_.ex.time.minute(), BINARY_CLOCK_MINUTES.withBrightness(maxBrightness_));
                        break;
                    case 2:
                        strip_.showBar(min(state_.ex.measurements.vcc, 420) - 340,80, BINARY_CLOCK_BATTERY.withBrightness(maxBrightness_));
                        break;
                }
                return;
            }
            case LightsEffect::Breathe: {
                strip_.fill(effectColor_.withBrightness(effectCounter_ & 0xff));
                break;
            }
            case LightsEffect::BreatheBar: {
                //strip_.centeredBar(effectColor_, effectCounter_ & 0xff, 255);
                strip_.showBarCentered((effectCounter_ & 0xff) / 4, 64, effectColor_);
                break;
            }
            case LightsEffect::KnightRider: {
                strip_.showPoint((effectCounter_ & 0xff) / 4, 64, effectColor_);
                break;
            }
            // solid color that simply fills the whole strip with the effect color
            case LightsEffect::SolidColor: {
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

    static void audioLightsTick(uint8_t & step) {
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
            max(effect_.audio.maxDelta - effect_.audio.minDelta, 8),
            effectColor_
        );
        // fade the delta range at 1/32 the speed
        if (tickCountdown_ % 32 == 0) {
            if (effect_.audio.maxDelta > d)
                --effect_.audio.maxDelta;
            if (effect_.audio.minDelta < d)
                ++effect_.audio.minDelta;
        }
        if (effect_.audio.lastDelta < d)
            step = 255;
        else 
            step = max(1, (effect_.audio.maxDelta - effect_.audio.minDelta) / 4);
        effect_.audio.lastDelta = d;
    }

    /** Determines if there are any notifications and displays them over the strip values. 
     
        
     */
    static void lightsNotifications() {
        if (state_.state.wifiStatus() == WiFiStatus::Connecting) {
            neopixels_.showBarCentered(tickCountdown_ % 128, 128, Color::Blue().withBrightness(maxBrightness_));
        } else {
            bool draw = state_.state.charging() 
                       || (state_.state.mode() == Mode::Lights)
                       || (state_.ex.measurements.vcc < BATTERY_LOW_VCC)
                       || (state_.state.wifiStatus() != WiFiStatus::Off)
                       || (! state_.ex.walkieTalkie.isEmpty());
            if (! draw)
                return;
            neopixels_[0] = Color::Black();
            neopixels_[7] = Color::Black();
            switch (tickCountdown_ % 64) {
                case 0:
                case 1:
                case 2:
                    if (state_.state.wifiStatus() == WiFiStatus::Connected)
                        neopixels_[0] = Color::Blue().withBrightness(maxBrightness_);
                    else if (state_.state.wifiStatus() == WiFiStatus::AP)
                        neopixels_[0] = Color::Cyan().withBrightness(maxBrightness_);
                    break;
                case 16:
                case 17:
                case 18:
                    if (state_.state.mode() == Mode::Lights)
                        neopixels_[0] = MODE_COLOR_LIGHTS.withBrightness(maxBrightness_);
                    break;
                case 48:
                case 49:
                case 50:
                    if (state_.ex.measurements.vcc < BATTERY_LOW_VCC)
                        neopixels_[7] = Color::Red().withBrightness(maxBrightness_);
                    else if (state_.state.charging())
                        neopixels_[7] = Color::Green().withBrightness(maxBrightness_);
                    break;
                case 32:
                case 33:
                case 34:
                    if (! state_.ex.walkieTalkie.isEmpty())
                        neopixels_[0] = Color::Yellow().withBrightness(maxBrightness_);
                    break;
                default:
                    break;
            }
        }
    }

    static void showByte(uint8_t value, Color const & color) {
        strip_[7] = (value & 1) ? color : Color::Black();
        strip_[6] = (value & 2) ? color : Color::Black();
        strip_[5] = (value & 4) ? color : Color::Black();
        strip_[4] = (value & 8) ? color : Color::Black();
        strip_[3] = (value & 16) ? color : Color::Black();
        strip_[2] = (value & 32) ? color : Color::Black();
        strip_[1] = (value & 64) ? color : Color::Black();
        strip_[0] = (value & 128) ? color : Color::Black();
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
            uint8_t lastDelta;
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
        cli();
        // turn I2C off in case it was running before
        TWI0.MCTRLA = 0;
        TWI0.SCTRLA = 0;
        // reset I2C buffers and state
        i2cTxLength_ = 0;
        i2cTxBuffer_ = nullptr;
        i2cRxOffset_ = 0;
        recordingWrite_ = 0;
        recordingRead_ = 0;
        status_.i2cRxReady = false;
        // make sure that the pins are nout out - HW issue with the chip, will fail otherwise
        PORTB.OUTCLR = 0x03; // PB0, PB1
        // set the address and disable general call, disable second address and set no address mask (i.e. only the actual address will be responded to)
        TWI0.SADDR = AVR_I2C_ADDRESS << 1;
        TWI0.SADDRMASK = 0;
        // enable the TWI in slave mode, enable all interrupts
        TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm  | TWI_ENABLE_bm;
        // bus Error Detection circuitry needs Master enabled to work 
        // not sure why we need it
        TWI0.MCTRLA = TWI_ENABLE_bm;   
        sei();
    }
    
    /** Sets the IRQ to inform ESP that there is a state change it should attend to. 
     
        The IRQ is only set if ESP is altready powered on as otherwise pulling the AVR_IRQ pin low would mean next ESP boot will not be in normal mode. 
     */
    static void setIrq() {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (! status_.irq && digitalRead(DCDC_PWR) == DCDC_PWR_ON && state_.state.mode() != Mode::Sync) {
                status_.irq = true;
                pinMode(AVR_IRQ, OUTPUT);
                digitalWrite(AVR_IRQ, LOW);
                // if the ESP is not busy, set the IRQ timeout
                if (!status_.espBusy)
                    irqCountdown_ = IRQ_RESPONSE_TIMEOUT;
            }
        }
    }

    static void clearIrq(bool force = false) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (status_.irq) {
                pinMode(AVR_IRQ, INPUT);
                status_.irq = false;
                // reset the WDT as this only happened because of some ESP action
                wdt_reset();
                // let the countdown to reach 0 naturally
            }
        }
    }

    /** Hard-resets the ESP8266. 
     */
    static void resetESP() {
        LOG("Resetting ESP");
        peripheralPowerOff();
        clearIrq(true); // this is necessary so that ESP boots into a normal mode
        state_.state.setEspReset(true);
        state_.state.setWiFiStatus(WiFiStatus::Off);
        if (status_.recording)
            stopAudioCapture();
        status_.espBusy = false;
        // reinitialize I2C so that there are no issues with some pending state
        initializeI2C();
        delay(200); // enough delay to drain 3V3 capacitors so that the reset actually occurs
        peripheralPowerOn();
    }

    static void processCommand() {
        // process the I2C commands from ESP
        switch (i2cRxBuffer_[0]) {
            /* Sets the specified part of the extended state to the given values via a simple memcpy.
             */
            case msg::SetExtendedState::Id: {
                auto m = pointer_cast<msg::SetExtendedState*>(& i2cRxBuffer_);
                LOG("cmd SetExtendedState s: %u, o: %u", m->size, m->offset);
                uint8_t * target = pointer_cast<uint8_t*>(&state_.ex) + m->offset;
                memcpy(target, i2cRxBuffer_ + sizeof(msg::SetExtendedState), m->size);
                break;
            }
            case msg::SetSettings::Id: {
                auto m = pointer_cast<msg::SetSettings*>(& i2cRxBuffer_);
                LOG("cmd SetSettings");
                maxBrightness_ = m->maxBrightness;
                status_.radioEnabled = m->radioEnabled;
                status_.discoEnabled = m->discoEnabled;
                status_.lightsEnabled = m->lightsEnabled;
                syncHour_ = m->syncHour;
                break;
            }
            case msg::Sleep::Id: {
                LOG("cmd Sleep");
                if (status_.sync) {
                    status_.sync = false;
                    state_.state.setMode(Mode::Sync);
                    setIrq();
                } else {
                    status_.shouldSleep = true;
                }
                break;
            }
            /** Resets the AVR (software reset).
             */
            case msg::Reset::Id: {
                LOG("cmd reset");
                _PROTECTED_WRITE(RSTCTRL.SWRR,1);
                break;
            }
            case msg::AVRWatchdogReset::Id: {
                LOG("wdt_reset");
                wdt_reset();
                break;
            }
            case msg::SetMode::Id: {
                auto m = pointer_cast<msg::SetMode*>(& i2cRxBuffer_);
                LOG("cmd SetMode m: %u, %u", static_cast<uint8_t>(m->mode), static_cast<uint8_t>(m->musicMode));
                state_.state.setMode(m->mode);
                state_.state.setMusicMode(m->musicMode);
                if ((m->mode == Mode::Music || m->mode == Mode::Lights) && m->musicMode == MusicMode::Radio)
                    digitalWrite(AUDIO_SRC, AUDIO_SRC_RADIO);
                else 
                    digitalWrite(AUDIO_SRC, AUDIO_SRC_ESP);
                break;
            }
            case msg::SetIdle::Id: {
                auto m = pointer_cast<msg::SetIdle*>(& i2cRxBuffer_);
                LOG("cmd SetIdle, timeout: %u", m->timeout);
                state_.state.setIdle(m->idle);
                if (state_.state.idle())
                    startAudioCapture(AudioADCSource::Mic);
                    //stopAudioCapture();
                else
                    startAudioCapture(AudioADCSource::Audio);
                // reset the timeout countdown (argument in minutes, we are converting to seconds)
                powerCountdown_ = static_cast<uint16_t>(m->timeout) * 60;
                break;
            }
            case msg::SetControlRange::Id: {
                auto m = pointer_cast<msg::SetControlRange*>(& i2cRxBuffer_);
                LOG("cmd SetControlRange v: %u, m: %u", m->value, m->max);
                control_.setValues(m->value, m->max);
                state_.state.setControlValue(m->value);
                break;
            }
            case msg::SetVolumeRange::Id: {
                auto m = pointer_cast<msg::SetVolumeRange*>(& i2cRxBuffer_);
                LOG("cmd SetVolumeRange v: %u, m: %u", m->value, m->max);
                volume_.setValues(m->value, m->max);
                state_.state.setVolumeValue(m->value);
                break;
            }
            case msg::LightsFill::Id: {
                auto m = pointer_cast<msg::LightsFill*>(& i2cRxBuffer_);
                //LOG("cmd LightsFill rgb: %u,%u,%u, t: %u", m->color.r, m->color.g, m->color.b, m->timeout);
                strip_.fill(m->color);
                effectTimeout_ = m->timeout;
                break;
            }
            case msg::LightsPoint::Id: {
                auto m = pointer_cast<msg::LightsPoint*>(& i2cRxBuffer_);
                //LOG("cmd LightsPoint v: %u, m: %u, rgb: %u,%u,%u, t: %u", m->value, m->max, m->color.r, m->color.g, m->color.b, m->timeout);
                strip_.showPoint(m->value, m->max, m->color);
                effectTimeout_ = m->timeout;
                break;
            }
            case msg::LightsBar::Id: {
                auto m = pointer_cast<msg::LightsPoint*>(& i2cRxBuffer_);
                //LOG("cmd LightsBar v: %u, m: %u, rgb: %x%x%x, t: %u", m->value, m->max, m->color.r, m->color.g, m->color.b, m->timeout);
                strip_.showBar(m->value, m->max, m->color);
                effectTimeout_ = m->timeout;
                // as this serves as a progressbar, reset the ESP busy counter, if in esp busy mode
                if (status_.espBusy)
                    irqCountdown_ = ESP_BUSY_TIMEOUT;
                break;
            }
            case msg::LightsBarCentered::Id: {
                auto m = pointer_cast<msg::LightsPoint*>(& i2cRxBuffer_);
                //LOG("cmd LightsBarCentered v: %u, m: %u, rgb: %u,%u,%u, t: %u", m->value, m->max, m->color.r, m->color.g, m->color.b, m->timeout);
                strip_.showBarCentered(m->value, m->max, m->color);
                effectTimeout_ = m->timeout;
                break;
            }
            case msg::LightsColors::Id: {
                auto m = pointer_cast<msg::LightsColors*>(& i2cRxBuffer_);
                //LOG("cmd LightsColors, t: %u", m->timeout);
                for (uint8_t i = 0; i < 8; ++i)
                    strip_[i] = m->colors[i];
                effectTimeout_ = m->timeout;
                break;
            }
            case msg::StartRecording::Id: {
                auto m = pointer_cast<msg::StartRecording*>(& i2cRxBuffer_);
                LOG("cmd StartRecording");
                status_.recording = true;
                startAudioCapture(AudioADCSource::Mic);
                effectColor_ = m->color;
                break;            
            }
            case msg::StopRecording::Id: {
                LOG("cmd StopRecording");
                status_.recording = false;
                //state_.state.setVolumeButtonDown(false);
                stopAudioCapture();
                break;
            }
            case msg::SetWiFiStatus::Id: {
                auto m = pointer_cast<msg::SetWiFiStatus*>(& i2cRxBuffer_);
                LOG("cmd SetWiFiStatus %u", static_cast<uint8_t>(m->status));
                state_.state.setWiFiStatus(m->status);
                break;
            }
            case msg::SetESPBusy::Id: {
                auto m = pointer_cast<msg::SetESPBusy*>(& i2cRxBuffer_);
                LOG("cmd SetBusy %u", static_cast<uint8_t>(m->busy));
                status_.espBusy = m->busy;
                if (status_.espBusy)
                    irqCountdown_ = ESP_BUSY_TIMEOUT;
                else if (status_.irq)
                    irqCountdown_ = IRQ_RESPONSE_TIMEOUT;
                break;
            }
            default:
                // TODO what to do in such an error? 
                LOG("cmd unrecognized id %u", i2cRxBuffer_[0]);
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

    /** I2C transmissible data.
     */
    inline static AVRState state_;

//@}

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
        TCB0.CCMP = 1250; // for 8kHz
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
        stopAudioCapture(); // make sure we will not be interefered in the setup routine
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
    static void showErrorByte(uint8_t value, Color const & color) {
        cli();
        LOG("error: %u", value);
        showByte(value, color);
        neopixels_.moveTowardsReversed(strip_, 255);
        neopixels_.update();
        while (true) { 
            wdt_reset();
        };
    }
//@}

    inline static volatile struct {

        /** If true, AVR is currently sleeping.
         */
        bool sleeping : 1;

        /** Set to notify the main thread to go to sleep. 
         */
        bool shouldSleep : 1;

        /** If true, the IRQ flag is currently raised, which is a notification to ESP that it should read the state and react to the changes (or recording buffer if we are recording).
         */        
        bool irq : 1;

        /** If true, synchronization run is to be scheduled.
         */
        bool sync : 1;

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

        /** True if in the wakeup check, i.e. sleeping, but button down which might lead to eventual wakeup if the button is pressed long enough. While true, the RTC interrupt period is 1/64th of a second instead of the 1s when regularly sleeping.
         */
        bool wakeup : 1;

        /** Indicates that ESP is currently busy. 
         */
        bool espBusy : 1;

        bool radioEnabled : 1;
        bool discoEnabled : 1;
        bool lightsEnabled : 1;

    } status_;


    /** Hour at which the automated daily synchronization should be performed.
     */
    static inline uint8_t syncHour_ = 3;

    /** Max brightness of the LED strip. 
     */
    static inline uint8_t maxBrightness_ = 255;


    static inline uint8_t undervoltageCountdown_ = UNDERVOLTAGE_TIMEOUT;
    static inline uint16_t irqCountdown_ = 0;
    static inline uint8_t tickCountdown_ = 0; 
    /** Countdown to poweroff in seconds. 
     */
    static inline uint16_t powerCountdown_ = 60;
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
    //digitalWrite(DEBUG_PIN, HIGH);
    RTC.PITINTFLAGS = RTC_PI_bm;
    if (Player::status_.sleeping) {
        if (Player::status_.wakeup) {
            if ((++Player::tickCountdown_ % 64) == 0) 
                Player::state_.ex.time.secondTick();
            if (--Player::powerCountdown_ == 0)
                Player::status_.sleeping = false;
        } else {
            // do one second tick
            Player::secondTick();
        }
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
    //digitalWrite(DEBUG_PIN, LOW);
}

/** [ISR]
 */
ISR(TWI0_TWIS_vect) {
    //digitalWrite(DEBUG_PIN, HIGH);
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
                Player::state_.state.clearEvents();
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
            Player::i2cTxLength_ = sizeof(AVRState) - Player::i2cRxBuffer_[0];
            Player::i2cRxOffset_ = 0;
        // otherwise, if currently recording, prepare to send next 32 bytes of audio, if available        
        } else if (Player::status_.recording) {
            Player::i2cTxBuffer_ = (uint8_t *)(& Player::recordingBuffer_) + Player::recordingRead_;
            Player::i2cTxLength_ = (((Player::recordingRead_ + 32) & 0xff) <= Player::recordingWrite_) ? 32 : 0;
        // or if not recording, do the default action, which is to send the state
        } else {
            Player::i2cTxBuffer_ = (uint8_t*)(& Player::state_);
            Player::i2cTxLength_ = sizeof(AVRState);
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
    //digitalWrite(DEBUG_PIN, LOW);
}

ISR(ADC1_RESRDY_vect) {
    //digitalWrite(DEBUG_PIN, HIGH);
    Player::recordingBuffer_[Player::recordingWrite_++] = (ADC1.RES / 8) & 0xff;
    //digitalWrite(DEBUG_PIN, LOW);
    if (Player::recordingWrite_ % 32 == 0 && Player::status_.recording)
        Player::setIrq();
}

void setup() {
    Player::initialize();
}

void loop() {
    Player::loop();
}

#endif // actual code 
#endif // ARCH_ATTINY
