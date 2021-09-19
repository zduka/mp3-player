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
        events_ &= ~(CONTROL_PRESS_MASK | CONTROL_LONG_PRESS_MASK | VOLUME_PRESS_MASK | VOLUME_LONG_PRESS_MASK | DOUBLE_LONG_PRESS_MASK);
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

    /** Returns the current mode of the player. 
     
        When ESP starts, this is also the last mode that was used.
     */
    Mode mode() volatile {
        return static_cast<Mode>((events_ & MODE_MASK) >> 5);
    }

    void setMode(Mode value) volatile {
        events_ &= ~MODE_MASK;
        events_ |= (static_cast<uint8_t>(value) << 5) & MODE_MASK;
    }

private:
    static constexpr uint8_t CONTROL_PRESS_MASK = 1 << 0;
    static constexpr uint8_t CONTROL_LONG_PRESS_MASK = 1 << 1;
    static constexpr uint8_t VOLUME_PRESS_MASK = 1 << 2;
    static constexpr uint8_t VOLUME_LONG_PRESS_MASK = 1 << 3;
    static constexpr uint8_t DOUBLE_LONG_PRESS_MASK = 1 << 4;
    static constexpr uint8_t MODE_MASK = 7 << 5;
    uint8_t events_;
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

static_assert(sizeof(State) == 4);

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

class Settings {
public:

    bool mp3Enabled() const {
        return raw_ & ENABLE_MP3;
    }
    bool radioEnabled() const {
        return raw_ & ENABLE_RADIO;
    }
    bool walkieTalkieEnabled() const {
        return raw_ & ENABLE_WALKIE_TALKIE;
    }
    bool nightLightsEnabled() const {
        return raw_ & ENABLE_NIGHT_LIGHTS;
    }
    bool radioManualTuningEnabled() const {
        return raw_ & RADIO_ENABLE_MANUAL_TUNING;
    }

    bool radioForceMono() const {
        return raw_ & RADIO_FORCE_MONO;

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

private:
    static constexpr uint16_t ENABLE_MP3 = 1 << 0;
    static constexpr uint16_t ENABLE_RADIO = 1 << 1;
    static constexpr uint16_t ENABLE_WALKIE_TALKIE = 1 << 2;
    static constexpr uint16_t ENABLE_NIGHT_LIGHTS = 1 << 3;
    static constexpr uint16_t RADIO_ENABLE_MANUAL_TUNING = 1 << 4;
    static constexpr uint16_t RADIO_FORCE_MONO = 1 << 5;

    static constexpr uint16_t LOW_BATTERY = 1 << 11;
    static constexpr uint16_t MESSAGE_READY = 1 << 12;
    static constexpr uint16_t AP_ACTIVE = 1 << 13;
    static constexpr uint16_t NO_WIFI = 1 << 14;
    static constexpr uint16_t AIRPLANE_MODE = 1 << 15;

    uint16_t raw_;

} __attribute__((packed)); // Settings

static_assert(sizeof(Settings) == 2);


/** Settings for the MP3 mode. 
 */
class MP3Settings {
public:
    uint8_t numPlaylists;
    uint8_t playlistId;
    uint16_t trackId;

    void log() const {
        LOG("MP3 Settings:");
        LOG("    playlist: %u (total %u)", playlistId, numPlaylists);
        LOG("    track: %u", trackId);
    }
} __attribute__((packed)); // MP3Settings

static_assert(sizeof(MP3Settings) == 4);

/** Settings for the FM Radio mode. 
 */
class RadioSettings {
public:
    uint8_t stationId;
    uint16_t frequency;

    void log() const {
        LOG("Radio Settings:");
        LOG("    stationId: %u", stationId);
        LOG("    frequency: %u", frequency);
    }

} __attribute__((packed)); // RadioSettings

static_assert(sizeof(RadioSettings) == 3);

/** Settings for the Walkie-Talkie mode. 
 */
class WalkieTalkieSettings {
public:
    uint32_t updateId;

    void log() const {
        LOG("Walkie-Talkie Settings:");
        LOG("    updateId: %lli", updateId);
    }

} __attribute__((packed)); // WalkieTalkieSettings

static_assert(sizeof(WalkieTalkieSettings) == 4);

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

/** Settings for the night light mode. 
 */
class NightLightSettings {
public:
    static constexpr uint8_t HUE_RAINBOW = 32;
    NightLightEffect effect = NightLightEffect::KnightRider;
    uint8_t hue = HUE_RAINBOW;
    uint8_t maxBrightness = DEFAULT_BRIGHTNESS; 

    Color color() const {
        return Color::HSV(static_cast<uint16_t>(hue) << 11, 255, maxBrightness);
    }

    void log() const {
        LOG("Night Light Settings:");
        LOG("    effect: %u", effect);
        LOG("    hue: %u", hue);
        LOG("    maxBrightness: %u", maxBrightness);
    }

    
} __attribute__((packed)); // NightLightSettings

static_assert(sizeof(NightLightSettings) == 3);

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
    MP3Settings mp3Settings;
    RadioSettings radioSettings;
    WalkieTalkieSettings walkieTalkieSettings;
    NightLightSettings nightLightSettings;
    Notifications notifications;
    DateTime time;
    DateTime alarm;

    /** Returns the next mode. 
     
        This is usually straightforward unless the walkie-talkie mode is disabled. We need to know this inside the extended state as this function is used both by AVR and ESP and the extended state has enough information to determine whether a mode is active or not.
     */
    Mode getNextMode(Mode current) volatile {
        switch (current) {
            case Mode::MP3:
                return Mode::Radio;
            case Mode::Radio:
                return (walkieTalkieSettings.enabled) ? Mode::WalkieTalkie : Mode::NightLight;
            case Mode::WalkieTalkie:
                return Mode::NightLight;
            case Mode::NightLight:
            default:
                return (mp3Settings.numPlaylists > 0) ? Mode::MP3 : Mode::Radio;
        }
    }

    void log() const {
        measurements.log();
        mp3Settings.log();
        radioSettings.log();
        walkieTalkieSettings.log();
        nightLightSettings.log();
        notifications.log();
    }

} __attribute__((packed)); // ExtendedState

static_assert(sizeof(ExtendedState) == 29);




