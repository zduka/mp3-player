#pragma once

#include <stdint.h>

#include "helpers.h"
#include "color.h"
#include "datetime.h"

enum class Mode : uint8_t {
    MP3,
    Radio,
    WalkieTalkie,
    NightLight,
    // the alarm clock and birthday greeting modes are not directly accessible via controls, but are automatically selected by the system when appropriate
    AlarmClock,
    BirthdayGreeting,
    // Special mode for ESP to silently synchronize time & messages that is executed periodically when off
    Sync,
}; // Mode

enum class WiFiStatus : uint8_t {
    Off = 0,
    Connecting = 1,
    Connected = 2,
    SoftAP = 3
}; // WiFiStatus

/** Current state of the player controls 
 
    
 */
class State {
/** \name Peripherals register
 */
//@{
public:

    bool controlButtonDown() const volatile {
        return peripherals_ & CONTROL_DOWN_MASK;
    }

    bool volumeButtonDown() const volatile {
        return peripherals_ & VOLUME_DOWN_MASK;
    }

    bool headphonesConnected() const volatile {
        return peripherals_ & HEADPHONES_MASK;
    }

    bool charging() const volatile {
        return peripherals_ & CHARGING_MASK;
    }

    bool batteryMode() const volatile {
        return peripherals_ & BATTERY_MASK;
    }

    WiFiStatus wifiStatus() const volatile {
        return static_cast<WiFiStatus>((peripherals_ & WIFI_STATUS_MASK) >> 6);
    }

    void setControlButtonDown(bool value = true) volatile {
        if (value)
            peripherals_ |= CONTROL_DOWN_MASK;
        else
            peripherals_ &= ~CONTROL_DOWN_MASK;
    }

    void setVolumeButtonDown(bool value = true) volatile {
        if (value)
            peripherals_ |= VOLUME_DOWN_MASK;
        else
            peripherals_ &= ~VOLUME_DOWN_MASK;
    }

private:
    static constexpr uint8_t CONTROL_DOWN_MASK = 1 << 0;
    static constexpr uint8_t VOLUME_DOWN_MASK = 1 << 1;
    static constexpr uint8_t HEADPHONES_MASK = 1 << 2;
    static constexpr uint8_t CHARGING_MASK = 1 << 3;
    static constexpr uint8_t BATTERY_MASK = 1 << 4;
    static constexpr uint8_t LOW_BATTERY_MASK = 1 << 5;
    static constexpr uint8_t WIFI_STATUS_MASK = 3 << 6;
    volatile uint8_t peripherals_; 
//@}
/** \name Events register. 
 */
//@{
public:
    bool controlButtonPress() const volatile {
        return events_ & CONTROL_PRESS_MASK;
    }

    bool controlButtonLongPress() const volatile {
        return events_ & CONTROL_LONG_PRESS_MASK;
    }

    bool volumeButtonPress() const volatile {
        return events_ & VOLUME_PRESS_MASK;
    }

    bool volumeButtonLongPress() const volatile {
        return events_ & VOLUME_LONG_PRESS_MASK;
    }

    bool doubleButtonLongPress() const volatile {
        return events_ & DOUBLE_LONG_PRESS_MASK;
    }

    /** Clears the events flags. 
     
        This is called automatically every time the state is transmitted so that new events can occur. 
     */ 
    void clearEvents() volatile {
        events_ = 0;
    }

    void setControlButtonPress(bool value = true) volatile {
        if (value)
            events_ |= CONTROL_PRESS_MASK;
        else
            events_ &= ~CONTROL_PRESS_MASK;
    }

    void setVolumeButtonPress(bool value = true) volatile {
        if (value)
            events_ |= VOLUME_PRESS_MASK;
        else
            events_ &= ~VOLUME_PRESS_MASK;
    }

    void setControlButtonLongPress(bool value = true) volatile {
        if (value)
            events_ |= CONTROL_LONG_PRESS_MASK;
        else
            events_ &= ~CONTROL_LONG_PRESS_MASK;
    }

    void setVolumeButtonLongPress(bool value = true) volatile {
        if (value)
            events_ |= VOLUME_LONG_PRESS_MASK;
        else
            events_ &= ~VOLUME_LONG_PRESS_MASK;
    }

    void setDoubleButtonLongPress(bool value = true) volatile {
        if (value)
            events_ |= DOUBLE_LONG_PRESS_MASK;
        else
            events_ &= ~DOUBLE_LONG_PRESS_MASK;
    }

    bool controlTurn() const volatile {
        return events_ & CONTROL_TURN_MASK;
    }

    void setControlTurn(bool value = true) volatile {
        if (value)
            events_ |= CONTROL_TURN_MASK;
        else
            events_ &= ~CONTROL_TURN_MASK;
    }

    bool volumeTurn() const volatile {
        return events_ & VOLUME_TURN_MASK;
    }

    void setVolumeTurn(bool value = true) volatile {
        if (value)
            events_ |= VOLUME_TURN_MASK;
        else
            events_ &= ~VOLUME_TURN_MASK;
    }

private:
    static constexpr uint8_t CONTROL_PRESS_MASK = 1 << 0;
    static constexpr uint8_t CONTROL_LONG_PRESS_MASK = 1 << 1;
    static constexpr uint8_t VOLUME_PRESS_MASK = 1 << 2;
    static constexpr uint8_t VOLUME_LONG_PRESS_MASK = 1 << 3;
    static constexpr uint8_t DOUBLE_LONG_PRESS_MASK = 1 << 4;
    static constexpr uint8_t CONTROL_TURN_MASK = 1 << 5;
    static constexpr uint8_t VOLUME_TURN_MASK = 1 << 6;
    uint8_t events_;
//@}

/** \name Mode 
 */
//@{

public:
    /** Returns the current mode of the player. 
     
        When ESP starts, this is also the last mode that was used.
     */
    Mode mode() const volatile {
        return static_cast<Mode>(mode_ & MODE_MASK);
    }

    void setMode(Mode value) volatile {
        mode_ &= ~MODE_MASK;
        mode_ |= static_cast<uint8_t>(value) & MODE_MASK;
    }

    /** Returns true if the player is idle. 
     
        This means no playback / recording is happening. 
     */
    bool idle() const volatile {
        return mode_ & IDLE_MASK;
    }

    void setIdle(bool value) volatile {
        if (value) 
            mode_ |= IDLE_MASK;
        else
            mode_ &= ~IDLE_MASK;
    }

    /** Returns true if the current power on state is the initial state, i.e. if the AVR chip has also been power on as well (such as when batteries were inserted, or recharged from BOD) as opposed to wake up from sleep. 
     */
    bool initialPowerOn() const volatile {
        return mode_ & INITIAL_POWER_ON_MASK;
    }

    void setInitialPowerOn(bool value) volatile {
        if (value)
            mode_ |= INITIAL_POWER_ON_MASK;
        else
            mode_ &= ~INITIAL_POWER_ON_MASK;
    }

private:
    static constexpr uint8_t MODE_MASK = 7;
    static constexpr uint8_t IDLE_MASK = 8;
    static constexpr uint8_t INITIAL_POWER_ON_MASK = 16;

    uint8_t mode_;


//@}

/** \name Control and Volume knob values. 
 */
//@{
public:
    /** Value of the control knob. 
     
        Values from 0 to 1023 (inclusive) are supported. 
     */
    uint16_t controlValue() const {
        return knobValues_ & CONTROL_KNOB_MASK;
    }

    void setControlValue(uint16_t value) volatile {
        knobValues_ &= ~CONTROL_KNOB_MASK;
        knobValues_ |= value & CONTROL_KNOB_MASK;
    }

    /** Value of the volume knob. 
     
        Values from 0 to 63 (inclusive) are supported.
     */
    uint16_t volumeValue() const {
        return (knobValues_ & VOLUME_KNOB_MASK) >> 10;
    }

    void setVolumeValue(uint16_t value) volatile {
        value <<= 10;
        knobValues_ &= ~VOLUME_KNOB_MASK;
        knobValues_ |= value & VOLUME_KNOB_MASK;
    }

private:
    static constexpr uint16_t CONTROL_KNOB_MASK = 1023;
    static constexpr uint16_t VOLUME_KNOB_MASK = 63 << 10;
    uint16_t knobValues_;
//@}
} __attribute__((packed)); // State 

static_assert(sizeof(State) == 5);

/** Contains information about the temperature and voltage measurements done by AVR. 
 */
class Measurements {
public:
    /** The VCC voltage as measured on AVR multiplied by 100. 
     
        Supported values are 250 (2.5V, BOD) to 500 (5V, charger), other values are possible, but usually indicate an error. 
     */
    uint16_t vcc;

    /** The temperature measured by the AVR multiplied by 10. 
     
        Supported range is -300 (-30.0 C) to 800 (80 C)
     */
    int16_t temp;    

    void log() const {
        LOG("Measurements:");
        LOG("    vcc: %u", vcc);
        LOG("    temp: %i", temp);
    }
} __attribute__((packed)); // Measurements

static_assert(sizeof(Measurements) == 4);

/** Player settings. 
 */
class Settings {
public:

    Settings() {
        raw_ = ENABLE_MP3 | ENABLE_RADIO | ENABLE_WALKIE_TALKIE | ENABLE_NIGHT_LIGHT | RADIO_ENABLE_MANUAL_TUNING;
    }

    bool mp3Enabled() volatile {
        return raw_ & ENABLE_MP3;
    }

    bool radioEnabled() volatile {
        return raw_ & ENABLE_RADIO;
    }

    bool walkieTalkieEnabled() volatile {
        return raw_ & ENABLE_WALKIE_TALKIE;
    }

    bool NightLightEnabled() volatile {
        return raw_ & ENABLE_NIGHT_LIGHT;
    }

    bool radioManualTuningEnabled() volatile {
        return raw_ & RADIO_ENABLE_MANUAL_TUNING;
    }

    bool radioForceMono() volatile {
        return raw_ & RADIO_FORCE_MONO;
    }

    void setMp3Enabled(bool value) {
        raw_ = value ? (raw_ | ENABLE_MP3) : (raw_ & ~ENABLE_MP3);
    }

    void setRadioEnabled(bool value) {
        raw_ = value ? (raw_ | ENABLE_RADIO) : (raw_ & ~ENABLE_RADIO);
    }

    void setWalkieTalkieEnabled(bool value) {
        raw_ = value ? (raw_ | ENABLE_WALKIE_TALKIE) : (raw_ & ~ENABLE_WALKIE_TALKIE);
    }

    void setNightLightEnabled(bool value) {
        raw_ = value ? (raw_ | ENABLE_NIGHT_LIGHT) : (raw_ & ~ENABLE_NIGHT_LIGHT);
    }

    void setRadioManualTuningEnabled(bool value) {
        raw_ = value ? (raw_ | RADIO_ENABLE_MANUAL_TUNING) : (raw_ & ~RADIO_ENABLE_MANUAL_TUNING);
    }

    void setRadioForceMono(bool value) {
        raw_ = value ? (raw_ | RADIO_FORCE_MONO) : (raw_ & ~RADIO_FORCE_MONO);
    }

    void log() const {
        LOG("Settings:");
        LOG("    maxBrightness: %u", maxBrightness);
    }


    uint8_t maxBrightness = DEFAULT_BRIGHTNESS; 

private:
    static constexpr uint16_t ENABLE_MP3 = 1 << 0;
    static constexpr uint16_t ENABLE_RADIO = 1 << 1;
    static constexpr uint16_t ENABLE_WALKIE_TALKIE = 1 << 2;
    static constexpr uint16_t ENABLE_NIGHT_LIGHT = 1 << 3;
    static constexpr uint16_t RADIO_ENABLE_MANUAL_TUNING = 1 << 4;
    static constexpr uint16_t RADIO_FORCE_MONO = 1 << 5;

    uint16_t raw_;

} __attribute__((packed)); // Settings

static_assert(sizeof(Settings) == 3);


/** State for the MP3 mode. 
 */
class MP3State {
public:
    uint8_t playlistId;
    uint16_t trackId;

    void log() const {
        LOG("MP3 State:");
        LOG("    playlist: %u", playlistId);
        LOG("    track: %u", trackId);
    }
} __attribute__((packed)); // MP3State

static_assert(sizeof(MP3State) == 3);

/** State for the FM Radio mode. 
 */
class RadioState {
public:
    uint8_t stationId;
    uint16_t frequency;

    void log() const {
        LOG("Radio State:");
        LOG("    stationId: %u", stationId);
        LOG("    frequency: %u", frequency);
    }

} __attribute__((packed)); // RadioState

static_assert(sizeof(RadioState) == 3);

/** State of the Walkie-Talkie mode. 
 */
class WalkieTalkieState {
public:
    uint32_t updateId;

    void log() const {
        LOG("Walkie-Talkie State:");
        LOG("    updateId: %lli", updateId);
    }

} __attribute__((packed)); // WalkieTalkieState

static_assert(sizeof(WalkieTalkieState) == 4);

/** The various night light effects the player supports. 
 */
enum class NightLightEffect : uint8_t {
    Off = 0, 
    AudioLights, 
    Breathe, 
    BreatheBar,
    KnightRider, 
    StarryNight,
    SolidColor,
}; // NightLightEffect

/** State of the night light mode. 
 */
class NightLightState {
public:
    static constexpr uint8_t HUE_RAINBOW = 32;
    NightLightEffect effect = NightLightEffect::AudioLights;
    uint8_t hue = HUE_RAINBOW;

    void log() const {
        LOG("Night Light State:");
        LOG("    effect: %u", effect);
        LOG("    hue: %u", hue);
    }

    uint16_t colorHue() volatile {
        return (static_cast<uint16_t>(hue) << 11);
    }

    
} __attribute__((packed)); // NightLightState

static_assert(sizeof(NightLightState) == 2);

/** Active notifications. 
 */
class Notifications {
public:
    /** Returns true if there is at least one notification to be displayed. 
     */
    bool active() const {
        return raw_ != 0;
    }

    bool lowBattery() const {
        return raw_ & LOW_BATTERY;
    }

    bool messageReady() const {
        return raw_ & MESSAGE_READY;
    }

    bool apActive() const {
        return raw_ & AP_ACTIVE;
    }

    bool noWiFi() const {
        return raw_ & NO_WIFI;
    }

    bool airplaneMode() const {
        return raw_ & AIRPLANE_MODE;
    }

    void log() const {
        LOG("Notifications:");
        if (lowBattery())
            LOG("    low battery");
        if (messageReady())
            LOG("    message ready");
        if (apActive())
            LOG("    AP active");
        if (noWiFi())
            LOG("    no WiFi");
        if (airplaneMode())
            LOG("    airplane mode");
    }
private:
    static constexpr uint8_t LOW_BATTERY = 1;
    static constexpr uint8_t MESSAGE_READY = 2;
    static constexpr uint8_t AP_ACTIVE = 4;
    static constexpr uint8_t NO_WIFI = 8;
    static constexpr uint8_t AIRPLANE_MODE = 16;

    uint8_t raw_;
} __attribute__((packed)); // Notifications

static_assert(sizeof(Notifications) == 1);




class ExtendedState {
public:
    Measurements measurements; 
    Settings settings;
    MP3State mp3;
    RadioState radio;
    WalkieTalkieState walkieTalkie;
    NightLightState nightLight;
    Notifications notifications;
    DateTime time;
    DateTime alarm;

    /** Returns the next mode. 
     
        This is usually straightforward unless the walkie-talkie mode is disabled. We need to know this inside the extended state as this function is used both by AVR and ESP and the extended state has enough information to determine whether a mode is active or not.
     */
    Mode getNextMode(Mode current) volatile {

        switch (current) {
            case Mode::MP3:
                if (settings.radioEnabled()) 
                    return Mode::Radio;
            case Mode::Radio:
                if (settings.walkieTalkieEnabled())
                    return Mode::WalkieTalkie;
            case Mode::WalkieTalkie:
                if (settings.NightLightEnabled())
                    return Mode::NightLight;
            case Mode::NightLight:
            default:
                if (settings.mp3Enabled())
                    return Mode::MP3;
                else if (settings.radioEnabled())
                    return Mode::Radio;
                else if (settings.walkieTalkieEnabled())
                    return Mode::WalkieTalkie;
                else //if (settings.NightLightEnabled()) // ignore this check as we have to return something
                    return Mode::NightLight;
        }
    }

    void log() const {
        measurements.log();
        settings.log();
        mp3.log();
        radio.log();
        walkieTalkie.log();
        nightLight.log();
        notifications.log();
    }

} __attribute__((packed)); // ExtendedState

static_assert(sizeof(ExtendedState) == 28);
