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

    bool controlButtonDown() const {
        return peripherals_ & CONTROL_DOWN_MASK;
    }

    bool volumeButtonDown() const {
        return peripherals_ & VOLUME_DOWN_MASK;
    }

    bool headphonesConnected() const {
        return peripherals_ & HEADPHONES_MASK;
    }

    bool charging() const {
        return peripherals_ & CHARGING_MASK;
    }

    bool batteryMode() const {
        return peripherals_ & BATTERY_MASK;
    }

    WiFiStatus wifiStatus() const {
        return static_cast<WiFiStatus>((peripherals_ & WIFI_STATUS_MASK) >> 6);
    }

    void setControlButtonDown(bool value = true) {
        if (value)
            peripherals_ |= CONTROL_DOWN_MASK;
        else
            peripherals_ &= ~CONTROL_DOWN_MASK;
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
    bool controlButtonPress() const {
        return events_ & CONTROL_PRESS_MASK;
    }

    bool controlButtonLongPress() const {
        return events_ & CONTROL_LONG_PRESS_MASK;
    }

    bool volumeButtonPress() const {
        return events_ & VOLUME_PRESS_MASK;
    }

    bool volumeButtonLongPress() const {
        return events_ & VOLUME_LONG_PRESS_MASK;
    }

    /** Clears the events flags. 
     
        This is called automatically every time the state is transmitted so that new events can occur. 
     */ 
    void clearEvents() volatile {
        events_ &= ~(CONTROL_PRESS_MASK | CONTROL_LONG_PRESS_MASK | VOLUME_PRESS_MASK | VOLUME_LONG_PRESS_MASK);
    }

    /** Returns the current mode of the player. 
     
        When ESP starts, this is also the last mode that was used.
     */
    Mode mode() const {
        return static_cast<Mode>((events_ & MODE_MASK) >> 5);
    }

private:
    static constexpr uint8_t CONTROL_PRESS_MASK = 1 << 0;
    static constexpr uint8_t CONTROL_LONG_PRESS_MASK = 1 << 1;
    static constexpr uint8_t VOLUME_PRESS_MASK = 1 << 2;
    static constexpr uint8_t VOLUME_LONG_PRESS_MASK = 1 << 3;
    // 1 bit free
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

    /** Value of the volume knob. 
     
        Values from 0 to 63 (inclusive) are supported.
     */
    uint16_t volumeValue() const {
        return (knobValues_ & VOLUME_KNOB_MASK) >> 10;
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
     
        Supported values are 250 (2.5V, BOD) to 500 (5V, charger).
     */
    uint16_t vcc;

    /** The temperature measured by the AVR multiplied by 10. 
     
        Supported range is -300 (-30.0 C) to 800 (80 C)
     */
    int16_t temp;    
} __attribute__((packed)); // Measurements

static_assert(sizeof(Measurements) == 4);

/** Settings for the MP3 mode. 
 */
class MP3Settings {
public:
    uint8_t playlistId;
    uint16_t trackId;
} __attribute__((packed)); // MP3Settings

static_assert(sizeof(MP3Settings) == 3);

/** Settings for the FM Radio mode. 
 */
class RadioSettings {
public:
    uint8_t stationId;
    uint16_t frequency;
} __attribute__((packed)); // MP3Settings

static_assert(sizeof(RadioSettings) == 3);

/** Settings for the Walkie-Talkie mode. 
 */
class WalkieTalkieSettings {
public:
    uint32_t updateId;
} __attribute__((packed)); // WalkieTalkieSettings

static_assert(sizeof(WalkieTalkieSettings) == 4);

/** The various night light effects the player supports. 
 */
enum class NightLightEffect : uint8_t {
    Color, // single color
    Breathe, // breathing effect with single color, whole strip
    BreatheBar, // breathing effect, centered bar, single color
    KnightRider, // larson scanner, single color
    Running, // running lights, single color
    Sentinel // sentinel value indicating the end of effects
}; // NightLightEffect

/** Settings for the night light mode. 
 */
class NightLightSettings {
public:
    NightLightEffect effect;
    uint8_t hue;
} __attribute__((packed)); // NightLightSettings

static_assert(sizeof(NightLightSettings) == 2);
