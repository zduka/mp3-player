#pragma once

#define assert(...) 

#define AVR_I2C_ADDRESS 42

/** Brightness for notifications such as wifi connection, or low battery.
 */
#define NOTIFICATION_BRIGHTNESS 16


#define DEFAULT_POWEROFF_TIMEOUT 60
#define DEFAULT_WIFI_TIMEOUT 60
#define DEFAULT_NIGHT_LIGHTS_TIMEOUT 3600
#define DEFAULT_ALLOW_RADIO_MANUAL_TUNING true
#define DEFAULT_VOLUME 4
#define DEFAULT_MAX_SPEAKER_VOLUME 15
#define DEFAULT_MAX_HEADPHONES_VOLUME 15

#define ESP_VOLUME_STEP 0.1

#define BUTTON_LONG_PRESS_TICKS 64
#define SPECIAL_LIGHTS_TIMEOUT 32
#define IRQ_MAX_DELAY 32
#define RADIO_FREQUENCY_OFFSET 760
#define RADIO_FREQUENCY_MAX 320

#define DEFAULT_ACCENT_COLOR Color::White()
#define BUTTONS_COLOR Color::DarkRed()
#define BUTTONS_LONG_PRESS_COLOR Color::Red()
#define DEFAULT_VOLUME_COLOR Color::Yellow()
#define RADIO_STATION_COLOR Color::Green()
#define RADIO_FREQUENCY_COLOR Color::Cyan()
#define MP3_TRACK_COLOR Color::Blue()
#define MP3_PLAYLIST_COLOR Color::Purple()

#define DEFAULT_AP_SSID "mp3-player"
#define DEFAULT_AP_PASSWORD "mp3-player"


#include "helpers.h"
#include "color.h"
#include "datetime.h"


namespace msg {
    class SetMode;
    class SetWiFiStatus;
    class SetAudioSource;
    class SetMP3Settings;
    class SetRadioSettings;
    class SetNightLightSettings;
 } // namespace msg forward declarations

enum class Mode : uint8_t {
    MP3,
    Radio,
    WalkieTalkie,
    NightLight,
}; // Mode

enum class AudioSource : uint8_t {
    ESP = 0,
    Radio = 1
}; // AudioSource

enum class WiFiStatus : uint8_t {
    Off = 0,
    Connecting = 1,
    Connected = 2,
    SoftAP = 3
}; // WiFiStatus

enum class MP3Selection : uint8_t {
    Track = 0, 
    Playlist = 1,
}; // MP3Selection

enum class RadioTuning : uint8_t {
    Presets = 0,
    Manual = 1
}; // RadioTuning

/** The various night light effects the player supports. 
 */
enum class NightLightEffect : uint8_t {
    Color, // single color
    Breathe, // breathing effect with single color, whole strip
    BreatheBar, // breathing effect, centered bar, single color
    KnightRider, // larson scanner, single color
    Running, // running lights, single color
}; // NightLightEffect

enum class NightLightControlMode : uint8_t {
    Effect,
    Speed,
    Color
}; // NightLightControlMode

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

/** \name Events
 
    Contains information about the events that triggered the state update (when coming from AVR to ESP). When an event is active, the relevant part of the status register then contains the actual value, where applicable.
 */
//@{
public:

    bool chargingChange() const {
        return events_ & CHARGING_CHANGE_MASK;
    }

    bool headphonesChange() const {
        return events_ & HEADPHONES_CHANGE_MASK;
    }

    bool controlChange() const {
        return events_ & CONTROL_CHANGE_MASK;
    }

    bool controlButtonChange() const {
        return events_ & CONTROL_BUTTON_CHANGE_MASK;
    }

    bool controlPress() const {
        return events_ & CONTROL_PRESS_MASK;
    }

    bool controlLongPress() const {
        return events_ & CONTROL_LONG_PRESS_MASK;
    }

    bool volumeChange() const {
        return events_ & VOLUME_CHANGE_MASK;
    }

    bool volumeButtonChange() const {
        return events_ & VOLUME_BUTTON_CHANGE_MASK;
    }

    bool volumePress() const {
        return events_ & VOLUME_PRESS_MASK;
    }

    bool volumeLongPress() const {
        return events_ & VOLUME_LONG_PRESS_MASK;
    }

    bool alarm() const {
        return events_ & ALARM_MASK;
    }

#if (defined ARCH_ATTINY)
    /** Clears the events. 
     
        This is to be called whenever the state with the events is sent to ESP so that we do not resend existing events. 
     */
    void clearEvents() {
        events_ = 0;
    }

    /** Clears any events coming from the buttons 
     * */
    void clearButtonEvents() {
        events_ &= ~(CONTROL_PRESS_MASK + CONTROL_LONG_PRESS_MASK + CONTROL_BUTTON_CHANGE_MASK +
                     VOLUME_PRESS_MASK + VOLUME_LONG_PRESS_MASK + VOLUME_BUTTON_CHANGE_MASK);
    }

    void setControlPress() {
        events_ |= CONTROL_PRESS_MASK;
    }

    void setControlLongPress() {
        events_ |= CONTROL_LONG_PRESS_MASK;
    }

    void setVolumePress() {
        events_ |= VOLUME_PRESS_MASK;
    }

    void setVolumeLongPress() {
        events_ |= VOLUME_LONG_PRESS_MASK;
    }
#endif

private:

    static constexpr uint16_t CHARGING_CHANGE_MASK = 1 << 0;
    static constexpr uint16_t HEADPHONES_CHANGE_MASK = 1 << 1;
    static constexpr uint16_t CONTROL_CHANGE_MASK = 1 << 2;
    static constexpr uint16_t CONTROL_BUTTON_CHANGE_MASK = 1 << 3;
    static constexpr uint16_t CONTROL_PRESS_MASK = 1 << 4;
    static constexpr uint16_t CONTROL_LONG_PRESS_MASK = 1 << 5;
    static constexpr uint16_t VOLUME_CHANGE_MASK = 1 << 6;
    static constexpr uint16_t VOLUME_BUTTON_CHANGE_MASK = 1 << 7;
    static constexpr uint16_t VOLUME_PRESS_MASK = 1 << 8;
    static constexpr uint16_t VOLUME_LONG_PRESS_MASK = 1 << 9;
    static constexpr uint16_t ALARM_MASK = 1 << 10;

    volatile uint16_t events_;    

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


/** \name Controls
 
    Contains the actual information about the control and volume knob values and the state of their buttons.
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

    /** Determines if the control knob is currently pressed. 
     */
    bool controlDown() const {
        return controlState_ & CONTROL_DOWN_MASK;
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

    /** Determines if the volume knob is currently pressed. 
     */
    bool volumeDown() const {
        return controlState_ & CONTROL_DOWN_MASK;
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
        events_ |= CONTROL_CHANGE_MASK;
    }

    void setControlDown(bool down) {
        if (down)
            controlState_ |= CONTROL_DOWN_MASK;
        else
            controlState_ &= ~CONTROL_DOWN_MASK;
        events_ |= CONTROL_BUTTON_CHANGE_MASK;
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
        events_ |= VOLUME_CHANGE_MASK;
    }

    void setVolumeDown(bool down) {
        if (down)
            controlState_ |= VOLUME_DOWN_MASK;
        else
            controlState_ &= ~VOLUME_DOWN_MASK;
        events_ |= VOLUME_BUTTON_CHANGE_MASK;
    }

private:
    static constexpr uint16_t CONTROL_MASK = 1023;
    static constexpr uint16_t VOLUME_MASK = 63 << 10;
    volatile uint16_t controlValues_ = 0;
    volatile uint16_t controlMaximums_ = 0;
    static constexpr uint8_t CONTROL_DOWN_MASK = 1;
    static constexpr uint8_t VOLUME_DOWN_MASK = 2;
    volatile uint8_t controlState_ = 0;
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

    AudioSource audioSource() const {
        return static_cast<AudioSource>((mode_ & AUDIO_SOURCE_MASK) >> 5);
    }

    void setAudioSource(AudioSource value) {
        mode_ &= ~AUDIO_SOURCE_MASK;
        mode_ |= (static_cast<uint8_t>(value) << 5) & AUDIO_SOURCE_MASK;
    }

    WiFiStatus wifiStatus() const {
        return static_cast<WiFiStatus>((mode_ & WIFI_STATUS_MASK) >> 6);
    }

    void setWiFiStatus(WiFiStatus value) {
        mode_ &= ~WIFI_STATUS_MASK;
        mode_ |= (static_cast<uint8_t>(value) << 6) & WIFI_STATUS_MASK;
    }






private:
    // we really need only 2 bits, but using 3 gives extra space for the future 
    static constexpr uint8_t MODE_MASK = 7;
    static constexpr uint8_t AUDIO_LIGHTS_MASK = 1 << 4;
    static constexpr uint8_t AUDIO_SOURCE_MASK = 1 << 5;
    static constexpr uint8_t WIFI_STATUS_MASK = 3 << 6;

    volatile uint8_t mode_ = 0;


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

    /** When true, control knob is used to select playlist, not track id. 
     */
    bool mp3PlaylistSelection() const {
        return mp3_ & PLAYLIST_SELECTION_MASK;
    }

    void setMp3PlaylistSelection(bool value) {
        if (value)
            mp3_ |= PLAYLIST_SELECTION_MASK;
        else 
            mp3_ &= ~PLAYLIST_SELECTION_MASK;            
    }


    // TODO allow playlist change? 
    static constexpr uint16_t PLAYLIST_SELECTION_MASK = 1 << 13;
    static constexpr uint16_t PLAYLIST_ID_MASK = 7 << 10;
    static constexpr uint16_t TRACK_ID_MASK = 1023;
    volatile uint16_t mp3_;

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

    bool radioManualTuning() const {
        return radio_ & MANUAL_TUNING_MASK;
    }

    void setRadioManualTuning(bool value) {
        if (value)
            radio_ |= MANUAL_TUNING_MASK;
        else
            radio_ &= ~MANUAL_TUNING_MASK;
    }

private:

    // TODO allow manual tuning ?
    static constexpr uint16_t MANUAL_TUNING_MASK = 1 << 12;
    static constexpr uint16_t STATION_MASK = 7 << 9;
    static constexpr uint16_t FREQUENCY_MASK = 511;
    volatile uint16_t radio_ = 0;

public:

    NightLightEffect nightLightEffect() const {
        return NightLightEffect::Color;
        return static_cast<NightLightEffect>(nightLight_ & NIGHT_LIGHT_EFFECT_MASK);
    }

    void setNightLightEffect(NightLightEffect effect) {
        nightLight_ &= ~NIGHT_LIGHT_EFFECT_MASK;
        nightLight_ |= static_cast<uint16_t>(effect) & NIGHT_LIGHT_EFFECT_MASK;
    }

    uint16_t nightLightSpeed() const {
        return 1;
        return (nightLight_ & NIGHT_LIGHT_SPEED_MASK) >> 3;
    }

    void setNightLightSpeed(uint16_t value) {
        nightLight_ &= ~NIGHT_LIGHT_SPEED_MASK;
        nightLight_ |= (value << 3) & NIGHT_LIGHT_SPEED_MASK;
    }

    bool nightLightRainbow() const {
        return true;
        return nightLight_ & NIGHT_LIGHT_RAINBOW_MASK;
    }

    void nightLightSetRainbow(bool value) {
        if (value)
            nightLight_ |= NIGHT_LIGHT_RAINBOW_MASK;
        else
            nightLight_ &= ~NIGHT_LIGHT_RAINBOW_MASK;
    }

    NightLightControlMode nightLightControlMode() const {
        return static_cast<NightLightControlMode>((nightLight_ & NIGHT_LIGHT_CONTROL_MODE_MASK) >> 9);
    }

    void setNightLightControlMode(NightLightControlMode value) {
        nightLight_ &= ~NIGHT_LIGHT_CONTROL_MODE_MASK;
        nightLight_ |= (static_cast<uint16_t>(value) << 9) && NIGHT_LIGHT_CONTROL_MODE_MASK;
    }

    uint8_t nightLightHue() const {
        return nightLight_ >> 11;
    }

    void setNightLightHue(uint8_t value) {
        nightLight_ &= ~NIGHT_LIGHT_HUE_MASK;
        nightLight_ |= (value << 11) & NIGHT_LIGHT_HUE_MASK;
    }

    Color nightLightColor() const {
        return Color::White();
    }

private:

    static constexpr uint16_t NIGHT_LIGHT_EFFECT_MASK = 7;
    static constexpr uint16_t NIGHT_LIGHT_SPEED_MASK = 31 << 3;
    static constexpr uint16_t NIGHT_LIGHT_RAINBOW_MASK = 1 << 8;
    static constexpr uint16_t NIGHT_LIGHT_CONTROL_MODE_MASK = 3 << 9;
    static constexpr uint16_t NIGHT_LIGHT_HUE_MASK = 31 << 11;

    volatile uint16_t nightLight_ = 0;

//@}

} __attribute__((packed)); // State 
