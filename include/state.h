#pragma once

#include <stdint.h>

#include "helpers.h"
#include "color.h"
#include "datetime.h"

enum class MusicMode : uint8_t {
    MP3,
    Radio
};

/** Main mode. 
 */
enum class Mode : uint8_t {
    Music = 0,
    Disco = 1,
    Lights = 2,
    WalkieTalkie = 3,
    // the alarm clock and birthday greeting modes are not directly accessible via controls, but are automatically selected by the system when appropriate
    Alarm = 4,
    BirthdayGreeting = 5,
    // Special mode for ESP to silently synchronize time & messages that is executed periodically when off
    Sync = 6,

    /** Initial mode after power on of the AVR as opposed to wakeup. 
     */
    InitialPowerOn = 13,
    /** Signals the AVR to sleep.
     */
    Sleep = 14,
    /** Signals the ESP to poweroff so that AVR can go to sleep.
     */
    ESPOff = 15
}; // Mode

enum class WiFiStatus : uint8_t {
    Off = 0,
    Connecting = 1,
    Connected = 2,
    AP = 3
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

    void setHeadphonesConnected(bool value) volatile {
        if (value)
            peripherals_ |= HEADPHONES_MASK;
        else
            peripherals_ &= ~HEADPHONES_MASK;
    }

    void setCharging(bool value) volatile {
        if (value)
            peripherals_ |= CHARGING_MASK;
        else
            peripherals_ &= ~CHARGING_MASK;
    }

    void setBatteryMode(bool value) volatile {
        if (value)
            peripherals_ |= BATTERY_MASK;
        else
            peripherals_ &= ~BATTERY_MASK;
    }

    bool messageReady() const volatile {
        return peripherals_ & MESSAGE_READY_MASK;
    }

    void setMessageReady(bool value) volatile {
        if (value)
            peripherals_ |= MESSAGE_READY_MASK;
        else
            peripherals_ &= ~MESSAGE_READY_MASK;
    }

private:
    static constexpr uint8_t CONTROL_DOWN_MASK = 1 << 0;
    static constexpr uint8_t VOLUME_DOWN_MASK = 1 << 1;
    static constexpr uint8_t HEADPHONES_MASK = 1 << 2;
    static constexpr uint8_t CHARGING_MASK = 1 << 3;
    static constexpr uint8_t BATTERY_MASK = 1 << 4;
    static constexpr uint8_t LOW_BATTERY_MASK = 1 << 5;
    static constexpr uint8_t MESSAGE_READY_MASK = 1 << 6;
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

    /** Returns the music mode. 
     
        This is either mp3, or radio. 
     */
    MusicMode musicMode() const volatile {
        return static_cast<MusicMode>((mode_ & MUSIC_MODE_MASK) >> 4);
    }

    void setMusicMode(MusicMode mode) volatile {
        if (mode == MusicMode::MP3)
            mode_ &= ~ MUSIC_MODE_MASK;
        else
            mode_ |= MUSIC_MODE_MASK;
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


    /** WiFi mode. 
     */
    WiFiStatus wifiStatus() const volatile {
        return static_cast<WiFiStatus>((mode_ & WIFI_STATUS_MASK) >> 6);
    }

    void setWiFiStatus(WiFiStatus mode) volatile {
        mode_ &= ~WIFI_STATUS_MASK;
        mode_ |= (static_cast<uint8_t>(mode) << 6);
    }


private:
    static constexpr uint8_t MODE_MASK = 15;
    static constexpr uint8_t MUSIC_MODE_MASK = 1 << 4;
    static constexpr uint8_t IDLE_MASK = 1 << 5;
    static constexpr uint8_t WIFI_STATUS_MASK = 3 << 6;
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
enum class LightsEffect : uint8_t {
    Off = 0, 
    AudioLights, 
    Breathe, 
    BreatheBar,
    KnightRider, 
    StarryNight,
    BinaryClock,
    SolidColor,
}; // NightLightEffect

/** State of the night light mode. 
 */
class LightsState {
public:
    static constexpr uint8_t HUE_RAINBOW = 32;
    LightsEffect effect = LightsEffect::AudioLights;
    uint8_t hue = HUE_RAINBOW;

    void log() const {
        LOG("Lights State:");
        LOG("    effect: %u", effect);
        LOG("    hue: %u", hue);
    }

    uint16_t colorHue() volatile {
        return (static_cast<uint16_t>(hue) << 11);
    }

    
} __attribute__((packed)); // NightLightState

static_assert(sizeof(LightsState) == 2);

class ExtendedState {
public:
    Measurements measurements; 
    //Settings settings;
    MP3State mp3;
    RadioState radio;
    WalkieTalkieState walkieTalkie;
    LightsState lights;
    DateTime time;
    DateTime alarm;

    void log() const {
        measurements.log();
        //settings.log();
        mp3.log();
        radio.log();
        walkieTalkie.log();
        lights.log();
    }

} __attribute__((packed)); // ExtendedState

static_assert(sizeof(ExtendedState) == 24);

/** Returns the next music mode. 
 
    This is the current music mode if the mode is not music, or the other music mode (if available) and current mode is music. 
 */
inline MusicMode getNextMusicMode(Mode mode, MusicMode current, bool radioEnabled) {
    if (mode != Mode::Music)
        return current;
    if (current == MusicMode::MP3 && radioEnabled)
        return MusicMode::Radio;
    return MusicMode::MP3;
}

/** Settings stored on the SD card and used by the ESP alone. 
 
    TODO move to esp only
 */  
class ESPSettings {
public:

    bool speakerEnabled = true;
    uint8_t maxSpeakerVolume = 15;
    uint8_t maxHeadphonesVolume = 15;
    /** Timezone offset in seconds. 
     */
    int32_t timezone = 0;

    uint8_t maxBrightness = DEFAULT_BRIGHTNESS;

    /** If true, WiFi is not disconnected when leaving the walkie-talkie mode. 
     */
    bool keepWiFiAlive = true;

    bool radioEnabled = true;
    bool lightsEnabled = true;
    bool walkieTalkieEnabled = true;

}; // ESPSettings
