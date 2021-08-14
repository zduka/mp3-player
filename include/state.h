#pragma once

#define assert(...) 

#define AVR_I2C_ADDRESS 42


#define BATTERY_LOW 350
#define BATTERY_CRITICAL 340

/** Brightness for notifications such as wifi connection, or low battery.
 */
#define DEFAULT_NOTIFICATION_BRIGHTNESS 16
#define DEFAULT_BRIGHTNESS 32


#define DEFAULT_POWEROFF_TIMEOUT 60
#define DEFAULT_WIFI_TIMEOUT 3600
#define DEFAULT_NIGHT_LIGHTS_TIMEOUT 3600
#define DEFAULT_ALLOW_RADIO_MANUAL_TUNING true
#define DEFAULT_VOLUME 2
#define DEFAULT_MAX_SPEAKER_VOLUME 15
#define DEFAULT_MAX_HEADPHONES_VOLUME 15
#define DEFAULT_SPECIAL_LIGHTS_TIMEOUT 128

#define ESP_VOLUME_STEP 0.1

#define BUTTON_LONG_PRESS_TICKS 64

/** Number of ticks for which the long press progress bar is not displayed. 
 */
#define BUTTON_LONG_PRESS_DELAY 20
#define SPECIAL_LIGHTS_TIMEOUT 32
#define IRQ_MAX_DELAY 32
#define RADIO_FREQUENCY_OFFSET 760
#define RADIO_FREQUENCY_MAX 320

#define DEFAULT_ACCENT_COLOR Color::White()
#define BUTTONS_COLOR Color::DarkRed()
#define BUTTONS_LONG_PRESS_COLOR Color::Red()
#define VOLUME_COLOR Color::Yellow()
#define RADIO_STATION_COLOR Color::Green()
#define RADIO_FREQUENCY_COLOR Color::Cyan()
#define MP3_TRACK_COLOR Color::Blue()
#define MP3_PLAYLIST_COLOR Color::Purple()
#define NIGHTLIGHT_EFFECT_COLOR Color::White()

#define DEFAULT_AP_SSID "mp3-player"
#define DEFAULT_AP_PASSWORD "mp3-player"


#include "helpers.h"
#include "color.h"
#include "datetime.h"

/* Forward declarations for messages so that they can tap the internals of the state directly when transferring its parts from ESP to ATTiny. 
 */
namespace msg {
    class SetMode;
    class SetWiFiStatus;
    class SetAudioSource;
    class SetMP3Settings;
    class SetRadioSettings;
    class SetNightLightSettings;
 } // namespace msg

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

/** The player state. 
 
    The state is shared with both AVR and ESP via I2C bus. Getters and setters are used to both enforce platform independent storage and to make sure the stored compressed values can be retrieved in reasonable format. 

    State getters are available on both platforms, but different setters are provided for AVR and ESP. 

    - 
    - audio
    - control
    - rtc
    - lights
    - mp3 mode
    - radio mode

 */
class State {
    friend class msg::SetMode;
    friend class msg::SetWiFiStatus;
    friend class msg::SetAudioSource;
    friend class msg::SetMP3Settings;
    friend class msg::SetRadioSettings;
    friend class msg::SetNightLightSettings;
private:
    friend class Player;


/** \name Knob button states & events
 */
//@{
public:

    /** Determines if the control knob is currently pressed. 
     */
    bool controlDown() const {
        return controlState_ & CONTROL_DOWN_MASK;
    }

    void setControlDown(bool down) {
        if (down)
            controlState_ |= CONTROL_DOWN_MASK;
        else
            controlState_ &= ~CONTROL_DOWN_MASK;
    }

    /** Determines if the volume knob is currently pressed. 
     */
    bool volumeDown() const {
        return controlState_ & VOLUME_DOWN_MASK;
    }

    void setVolumeDown(bool down) {
        if (down)
            controlState_ |= VOLUME_DOWN_MASK;
        else
            controlState_ &= ~VOLUME_DOWN_MASK;
    }

    bool controlPress() const {
        return controlState_ & CONTROL_PRESS_MASK;
    }

    void setControlPress(bool value = true) {
        if (value)
            controlState_ |= CONTROL_PRESS_MASK;
        else
            controlState_ &= ~CONTROL_PRESS_MASK;
    }

    bool controlLongPress() const {
        return controlState_ & CONTROL_LONG_PRESS_MASK;
    }

    void setControlLongPress(bool value = true) {
        if (value)
            controlState_ |= CONTROL_LONG_PRESS_MASK;
        else
            controlState_ &= ~CONTROL_LONG_PRESS_MASK;
    }

    bool volumePress() const {
        return controlState_ & VOLUME_PRESS_MASK;
    }

    void setVolumePress(bool value = true) {
        if (value)
            controlState_ |= VOLUME_PRESS_MASK;
        else
            controlState_ &= ~VOLUME_PRESS_MASK;
    }

    bool volumeLongPress() const {
        return controlState_ & VOLUME_LONG_PRESS_MASK;
    }

    void setVolumeLongPress(bool value = true) {
        if (value)
            controlState_ |= VOLUME_LONG_PRESS_MASK;
        else
            controlState_ &= ~VOLUME_LONG_PRESS_MASK;
    }

    bool doubleLongPress() const {
        return controlState_ & DOUBLE_LONG_PRESS_MASK;
    }

    void setDoubleLongPress(bool value = true) {
        if (value)
            controlState_ |= DOUBLE_LONG_PRESS_MASK;
        else
            controlState_ &= ~DOUBLE_LONG_PRESS_MASK;
    }

    void clearButtonEvents() {
        controlState_ &= ~(CONTROL_PRESS_MASK | CONTROL_LONG_PRESS_MASK | VOLUME_PRESS_MASK | VOLUME_LONG_PRESS_MASK | DOUBLE_LONG_PRESS_MASK);
    }

private:

    static constexpr uint8_t CONTROL_DOWN_MASK = 1;
    static constexpr uint8_t VOLUME_DOWN_MASK = 2;
    static constexpr uint8_t CONTROL_PRESS_MASK = 4;
    static constexpr uint8_t VOLUME_PRESS_MASK = 8;
    static constexpr uint8_t CONTROL_LONG_PRESS_MASK = 16;
    static constexpr uint8_t VOLUME_LONG_PRESS_MASK = 32;
    static constexpr uint8_t DOUBLE_LONG_PRESS_MASK = 64;
    volatile uint8_t controlState_ = 0;
//@}

/** \name Knob values and ranges. 
 
    Contains the current values of the control & volume knobs and their allowed maximums (minimum is always 0). 
 */
//@{
public:
    /** The current value of the control knob. 
     
        Can be anywhere between 0 and maxControl() value, which may not exceed 1023. 
     */
    uint16_t control() const {
        return controlValues_ & CONTROL_MASK;
    }

    /** Maximum value (inclusive) the control knob can have, values up to 1023 are allowed. The minimum value is always 0. 
     */
    uint16_t maxControl() const {
        return controlMaximums_ & CONTROL_MASK;
    }

    /** The current value of the volume knob. 
     
        Can be anywhere between 0 and maxVolume() value which may not exceed 63. 
     */
    uint8_t volume() const {
        return (controlValues_ >> 10) & 63;
    }

    /** The maximum value of the volume knob. 
     
        Can be up to 63. Min value is always 0. 
     */
    uint8_t maxVolume() const {
        return (controlMaximums_ >> 10) & 63;
    }


    void resetControl(uint16_t value, uint16_t maxValue) {
        controlValues_ &= ~CONTROL_MASK;
        controlValues_ |= value;
        controlMaximums_ &= ~ CONTROL_MASK;
        controlMaximums_ |= maxValue;
    }

    void setControl(uint16_t value) {
        assert(value <= maxControl());
        controlValues_ &= ~CONTROL_MASK;
        controlValues_ |= value;
    }

    void resetVolume(uint8_t value, uint8_t maxValue) {
        controlValues_ &= ~VOLUME_MASK;
        controlValues_ |= value << 10;
        controlMaximums_ &= ~VOLUME_MASK;
        controlMaximums_ |= maxValue << 10;
    }
    
    void setVolume(uint8_t value) {
        assert(value < maxVolume());
        controlValues_ &= ~VOLUME_MASK;
        controlValues_ |= value << 10; 
    }

private:

    static constexpr uint16_t CONTROL_MASK = 1023;
    static constexpr uint16_t VOLUME_MASK = 63 << 10;
    volatile uint16_t controlValues_ = 0;
    volatile uint16_t controlMaximums_ = 0;
//@}



/** \name Peripherals
 
    Information about the peripherals managed by the AVR.
    */
//@{
public:
    /** Returns true if headphones are attached. 
     */
    bool headphones() const {
        return peripherals_ & HEADPHONES_MASK;
    }

    /** Returns true if the player is currently being charged. 
     */
    bool charging() const {
        return peripherals_ & CHARGING_MASK;
    }

    /** Returns the input voltage times 100. 

        As this is only useful to judge the battery capacity, or whether we are charhing, the following values may be returned:

        500 = charging
        420 .. 320 = battery level
        0 = below 
     */    
    uint16_t voltage() const {
        uint16_t result = (peripherals_ & VOLTAGE_MASK) >> 8;
        if (result == 63)
            result = 500;
        else if (result != 0)
            result = 320 + (result - 1) * 100 / 61;
        return result;
    }

    /** Returns the temperature measured by the onboard sensor in [C] * 10. 

        Has the precision of 0.25 degree over the entire range. 
     */
    int16_t temp() const {
        int16_t result = static_cast<int16_t>(peripherals_ & TEMP_MASK);
        result = result * 10 / 4;
        return -100 + result;
    }

#if (defined ARCH_ATTINY)
    /** Sets the voltage. 
        
        Takes the result of the adc and converts it to value from 0 to 127, where 0 means below 3.4V and 127 means 5V or above. 

        The input is the ADC measurement, which is the value for the internal 1.1V reference measured at the 0..VCC scale. The value is first converted to actual voltage * 100


         
     */
    void setVoltage(uint16_t value) {
        // convert value to voltage * 100, using the x = 1024 / value * 110 equation. Multiply by 512 only, then divide and multiply by two so that we always stay within uint16_t
        value = 110 * 512 / value;
        value = value * 2;
        // now update the value to fit the desired range
        if (value >= 320) {
            if (value > 430) 
                value = 63; // highest value, encodes 500
            else if (value > 420) 
                value = 62; // equivalent to 420
            else
                // value is in range 320..420, we need to map this to 1..62 (61 values)
                value = ((value - 320) * 61 / 100) + 1;
        } else {
            value = 0;
        }
        peripherals_ &= ~VOLTAGE_MASK;
        peripherals_ |= (value << 8);
    }

    void setTemp(uint16_t temp) {
        // drop 2 bits of precission so that we fit in 16bits
        int8_t sigrow_offset = SIGROW.TEMPSENSE1; // Read signed value from signature row
        uint8_t sigrow_gain = SIGROW.TEMPSENSE0;
        uint32_t x = (temp - sigrow_offset);
        x *= sigrow_gain;
        x += 0x80; // add 1/2 to get correct rounding
        // convert to celsius offset by 10 (263.15 K)
        if (x < 67336) {
            x = 0;
        } else {
            x -= 67336;
            if (x > 16320)
                x = 16320;
        }
        // now convert to celsius / 4
        x = x / 64;
        peripherals_ &= ~TEMP_MASK;
        peripherals_ |= (x & TEMP_MASK);
    }
#endif


private:
    static const uint16_t HEADPHONES_MASK = 0x1 << 15;
    static const uint16_t CHARGING_MASK = 0x1 << 14;
    static const uint16_t VOLTAGE_MASK = 0x3f << 8;
    static const uint16_t TEMP_MASK = 0xff;

    volatile uint16_t peripherals_ = 0;

//@}


/** \name Mode and mode information. 
 */
//@{
public:

    Mode mode() const {
        return static_cast<Mode>(mode_ & MODE_MASK);
    }

    void setMode(Mode mode) {
        mode_ &= ~MODE_MASK;
        mode_ |= static_cast<uint8_t>(mode);
    }

    bool audioLights() const {
        return mode_ & AUDIO_LIGHTS_MASK;
    }

    void setAudioLights(bool value) {
        if (value)
            mode_ |= AUDIO_LIGHTS_MASK;
        else 
            mode_ &= ~ AUDIO_LIGHTS_MASK;
    }


    WiFiStatus wifiStatus() const {
        return static_cast<WiFiStatus>((mode_ & WIFI_STATUS_MASK) >> 5);
    }

    void setWiFiStatus(WiFiStatus value) {
        mode_ &= ~WIFI_STATUS_MASK;
        mode_ |= (static_cast<uint8_t>(value) << 5) & WIFI_STATUS_MASK;
    }

private:
    // we really need only 2 bits, but using 3 gives extra space for the future 
    static constexpr uint8_t MODE_MASK = 7;
    static constexpr uint8_t AUDIO_LIGHTS_MASK = 1 << 4;
    static constexpr uint8_t WIFI_STATUS_MASK = 3 << 5;
    // 1 bit left

    volatile uint8_t mode_ = 0;
//@}

/** \name MP3 Mode Settings
 
    The mp3 mode remembers the current playlist & track id so that playback can resume where it left. 
 */
//@{
public:

    uint8_t mp3PlaylistId() const {
        return (mp3_ & PLAYLIST_ID_MASK) >> 10;
    }

    void setMp3PlaylistId(uint8_t value) {
        mp3_ &= ~ PLAYLIST_ID_MASK;
        mp3_ |= (value << 10) & PLAYLIST_ID_MASK;
    }

    uint16_t mp3TrackId() const {
        return mp3_ & TRACK_ID_MASK;
    }

    void setMp3TrackId(uint16_t value) {
        mp3_ &= ~TRACK_ID_MASK;
        mp3_ |= (value & TRACK_ID_MASK);
    }
private:

    // TODO allow playlist change? 
    static constexpr uint16_t PLAYLIST_ID_MASK = 7 << 10;
    static constexpr uint16_t TRACK_ID_MASK = 1023;
    volatile uint16_t mp3_;
//@}

/** \name Radio Settings. 
 
    Radio settings remember the preset station index and the manualy tuned frequency.
 */
//@{
public:

    uint16_t radioFrequency() const {
        return (radio_ & FREQUENCY_MASK) + RADIO_FREQUENCY_OFFSET;
    }

    void setRadioFrequency(uint16_t mhzx10) {
        radio_ &= ~ FREQUENCY_MASK;
        radio_ |= mhzx10 - RADIO_FREQUENCY_OFFSET;
    }

    uint8_t radioStation() const {
        return (radio_ & STATION_MASK) >> 9;
    }

    void setRadioStation(uint8_t index) {
        radio_ &= ~ STATION_MASK;
        radio_ |= (index << 9) & STATION_MASK;
    }

private:

    static constexpr uint16_t STATION_MASK = 7 << 9;
    static constexpr uint16_t FREQUENCY_MASK = 511;
    volatile uint16_t radio_ = 0;

//@}


/** \name Night-Lights Settings
 
    Remembers the effect being used and its color. Each effect has its own speed hardwired so is not part of the settings. 
 */
//@{
public:

    static constexpr uint8_t NIGHTLIGHT_WHITE_HUE = 0;
    static constexpr uint8_t NIGHTLIGHT_ACCENT_COLOR_HUE = 1; 
    static constexpr uint8_t NIGHTLIGHT_RAINBOW_HUE = 31;

    NightLightEffect nightLightEffect() const {
        return static_cast<NightLightEffect>(nightLight_ & NIGHT_LIGHT_EFFECT_MASK);
    }

    void setNightLightEffect(NightLightEffect effect) {
        nightLight_ &= ~NIGHT_LIGHT_EFFECT_MASK;
        nightLight_ |= static_cast<uint16_t>(effect) & NIGHT_LIGHT_EFFECT_MASK;
    }

    uint8_t nightLightHue() const {
        return (nightLight_ & NIGHT_LIGHT_HUE_MASK) >> 4;
    }

    void setNightLightHue(uint8_t value) {
        nightLight_ &= ~NIGHT_LIGHT_HUE_MASK;
        nightLight_ |= (value << 4) & NIGHT_LIGHT_HUE_MASK;
    }

    Color nightLightColor(Color const & accentColor, uint8_t brightness) const {
        uint8_t hue = nightLightHue();
        switch (hue) {
            case NIGHTLIGHT_WHITE_HUE:
            case NIGHTLIGHT_RAINBOW_HUE:
                return Color::White().withBrightness(brightness);
            case NIGHTLIGHT_ACCENT_COLOR_HUE:
                return accentColor.withBrightness(brightness);
            // actual hue angle from 0 to 28
            default:
                hue -= 2;
                return Color::HSV(hue * 2340, 255, brightness);
        }
    }

private:

    static constexpr uint16_t NIGHT_LIGHT_EFFECT_MASK = 15;
    static constexpr uint16_t NIGHT_LIGHT_HUE_MASK = 31 << 4;

    volatile uint16_t nightLight_ = 0;
//@}

/** \name Walkie-Talkie
 
    Walkie talkie remembers the latest update id. 
 */
//@{
public:
    uint32_t telegramBotLatestUpdate() const {
        return telegramBotLatestUpdate_;
    }

    void setTelegramBotLatestUpdate(uint32_t value) {
        telegramBotLatestUpdate_ = value;
    }

private:
    uint32_t telegramBotLatestUpdate_;
//@}

} __attribute__((packed)); // State 
