#pragma once

#define assert(...) 

#define AVR_I2C_ADDRESS 42

/** Brightness for notifications such as wifi connection, or low battery.
 */
#define NOTIFICATION_BRIGHTNESS 16


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

    bool audioLights() const {
        return mode_ & AUDIO_LIGHTS_MASK;
    }

    AudioSource audioSource() const {
        return static_cast<AudioSource>((mode_ & AUDIO_SOURCE_MASK) >> 5);
    }

    WiFiStatus wifiStatus() const {
        return static_cast<WiFiStatus>((mode_ & WIFI_STATUS_MASK) >> 6);
    }

    uint8_t mp3PlaylistId() const {
        return (mp3_ & PLAYLIST_ID_MASK) >> 10;
    }

    uint16_t mp3TrackId() const {
        return mp3_ & TRACK_ID_MASK;
    }

    /** When true, control knob is used to select playlist, not track id. 
     */
    bool mp3PlaylistSelection() const {
        return mp3_ & PLAYLIST_SELECTION_MASK;
    }

    uint16_t radioFrequency() const {
        return (radio_ & FREQUENCY_MASK) + RADIO_FREQUENCY_OFFSET;
    }

    uint8_t radioStation() const {
        return (radio_ & STATION_MASK) >> 9;
    }

    bool radioManualTuning() const {
        return radio_ & MANUAL_TUNING_MASK;
    }

    void setMode(Mode mode) {
        mode_ &= ~MODE_MASK;
        mode_ |= static_cast<uint8_t>(mode);
    }

    void setAudioLights(bool value) {
        if (value)
            mode_ |= AUDIO_LIGHTS_MASK;
        else 
            mode_ &= ~ AUDIO_LIGHTS_MASK;
    }

    void setAudioSource(AudioSource value) {
        mode_ &= ~AUDIO_SOURCE_MASK;
        mode_ |= (static_cast<uint8_t>(value) << 5) & AUDIO_SOURCE_MASK;
    }

    void setWiFiStatus(WiFiStatus value) {
        mode_ &= ~WIFI_STATUS_MASK;
        mode_ |= (static_cast<uint8_t>(value) << 6) & WIFI_STATUS_MASK;
    }

    void setMP3PlaylistId(uint8_t value) {
        mp3_ &= ~ PLAYLIST_ID_MASK;
        mp3_ |= (value << 10) & PLAYLIST_ID_MASK;
    }

    void setRadioFrequency(uint16_t mhzx10) {
        radio_ &= ~ FREQUENCY_MASK;
        radio_ |= mhzx10 - RADIO_FREQUENCY_OFFSET;
    }

    void setRadioStation(uint8_t index) {
        radio_ &= ~ STATION_MASK;
        radio_ |= (index << 9) & STATION_MASK;
    }

    void setRadioManualTuning(bool value) {
        if (value)
            radio_ |= MANUAL_TUNING_MASK;
        else
            radio_ &= ~MANUAL_TUNING_MASK;
    }

private:
    // we really need only 2 bits, but using 3 gives extra space for the future 
    static constexpr uint8_t MODE_MASK = 7;
    static constexpr uint8_t AUDIO_LIGHTS_MASK = 1 << 4;
    static constexpr uint8_t AUDIO_SOURCE_MASK = 1 << 5;
    static constexpr uint8_t WIFI_STATUS_MASK = 3 << 6;

    volatile uint8_t mode_ = 0;

    // TODO allow playlist change? 
    static constexpr uint16_t PLAYLIST_SELECTION_MASK = 1 << 13;
    static constexpr uint16_t PLAYLIST_ID_MASK = 7 << 10;
    static constexpr uint16_t TRACK_ID_MASK = 1023;
    volatile uint16_t mp3_;

    // TODO allow manual tuning ?
    static constexpr uint16_t MANUAL_TUNING_MASK = 1 << 12;
    static constexpr uint16_t STATION_MASK = 7 << 9;
    static constexpr uint16_t FREQUENCY_MASK = 511;
    volatile uint16_t radio_ = 0;

//@}

// colors

public:

    Color accentColor() const {
        return accentColor_;
    }

    void setAccentColor(Color const & value) {
        accentColor_ = value;
    }

    Color controlColor() const {
        return controlColor_;
    }

    void setControlColor(Color const & value) {
        controlColor_ = value;
    }

    Color volumeColor() const {
        return volumeColor_;
    }

    void setVolumeColor(Color const & value) {
        volumeColor_ = value;
    }

private:
    volatile Color accentColor_ = DEFAULT_ACCENT_COLOR;
    volatile Color controlColor_ = MP3_TRACK_COLOR;
    volatile Color volumeColor_ = DEFAULT_VOLUME_COLOR;

// mode backups



} __attribute__((packed)); // State 






#ifdef HAHA


/** We can't use bitfields as sadly, they are implementation specific. 

    The state is kept at AVR, which preserves it throughout power cycles as long as the battery is not completely depleted. 
 */
struct State {
public:

    /** \name State register.
     */
    //@{
    enum class Mode : uint8_t {
        /** Default mode where MP3 files from the SD card are played. 
         */
        MP3 = 0,
        /** Radio mode where ESP controls the RD8507 chip to play FM radio stations. 
         */
        Radio = 1,
        /** WWW Settings mode. 
         */
        WWW = 2,
        NightLight = 3,
        WalkieTalkie = 4,

    };

    Mode mode() const {
        return static_cast<Mode>(state_ & STATE_MODE);
    }
    
    void setMode(Mode mode) {
        state_ = (state_ & ~STATE_MODE) + static_cast<uint8_t>(mode);
    }

    bool volumeButtonDown() const {
        return state_ & STATE_VOL_BTN;
    }

    bool controlButtonDown() const {
        return state_ & STATE_CTRL_BTN;
    }

    bool idle() const {
        return state_ & STATE_IDLE;
    }

    void setIdle(bool value) {
        if (value)
            state_ |= STATE_IDLE;
        else
            state_ &= ~STATE_IDLE;
    }

    //@}

    /** \name Power Register. 
     */
    //@{

    /** Shows the current input voltage. 

        VCC voltage times 100. Values from 300 to 500 are expected. Internally, the state stores the ADC result. 
 
    */
    uint16_t voltage() const {
        uint16_t result = (power_ & POWER_VOLTAGE);
        return (result == 0) ? 0 : (result * 200 / 127 + 300);
    }

    void setVoltage(uint16_t value) {
        value = (value > 500) ? 500 : value;
        value = (value > 300) ? (value - 300) : value;
        value = (value * 127) / 200;
        power_ = (power_ &~ POWER_VOLTAGE) | (value & POWER_VOLTAGE);
    }

    bool charging() const {
        return power_ & POWER_CHARGING;
    }

    //@}

    /** \name Events Register. 
     */
    //@{

    bool alarm() const {
        return events_ & EVENT_ALARM;
    }
    
    bool rtcSync() const {
        return events_ & EVENT_RTC_SYNC;
    }

    bool volumePress() const {
        return events_ & EVENT_VOL_PRESS;
    }
    
    bool volumeLongPress() const {
        return events_ & EVENT_VOL_LONG_PRESS;
    }
    
    bool controlPress() const {
        return events_ & EVENT_CTRL_PRESS;
    }
    
    bool controlLongPress() const {
        return events_ & EVENT_CTRL_LONG_PRESS;
    }

    void clearEvents() {
        events_ = 0;
    }
    //@}

    /** \name Audio Register.
     */
    //@{
    enum class AudioSrc : uint8_t {
        ESP = 0,
        Radio = 1,
    };


    uint8_t audioVolume() const {
        return audio_ & AUDIO_AUDIO_VOLUME;
    }

    void setAudioVolume(uint8_t value) {
        audio_ = (audio_ & ~AUDIO_AUDIO_VOLUME) + (value & AUDIO_AUDIO_VOLUME);
    }

    AudioSrc audioSrc() const {
        if (audio_ & AUDIO_AUDIO_SRC)
            return AudioSrc::Radio;
        else
            return AudioSrc::ESP;
    }

    void setAudioSrc(AudioSrc src) {
        if (src == AudioSrc::Radio)
            audio_ |= AUDIO_AUDIO_SRC;
        else
            audio_ &= ~AUDIO_AUDIO_SRC;
    }

    bool audioHeadphones() const {
        return audio_ & AUDIO_HEADPHONES;
    }

    void setAudioHeadphones(bool value) {
        if (value)
            audio_ |= AUDIO_HEADPHONES;
        else
            audio_ &= ~AUDIO_HEADPHONES;
    }

    bool audioLights() const {
        return audio_ & AUDIO_LIGHTS;
    }

    void setAudioLights(bool value) {
        if (value)
            audio_ |= AUDIO_LIGHTS;
        else
            audio_ &= ~AUDIO_LIGHTS;
    }

    //@}

    /** \name Control Register. 
     */
    //@{
    uint16_t control() const {
        return control_;
    }

    void setControl(uint16_t value) {
        control_ = value;
    }

    //@}

    /** \name MP3 Mode Settings register. 
     */
    //@{

    uint16_t mp3TrackId() const {
        return mp3_ & MP3_TRACK_ID;
    }

    void setMp3TrackId(uint16_t value) {
        mp3_ = (mp3_ & ~MP3_TRACK_ID) | (value & MP3_TRACK_ID);
    }

    uint8_t mp3PlaylistId() const {
        return (mp3_ & MP3_PLAYLIST_ID) >> 10;
    }

    void setMp3PlaylistId(uint16_t value) {
        mp3_ = (mp3_ & ~MP3_PLAYLIST_ID) | ((value << 10) & MP3_PLAYLIST_ID);
    }

    bool mp3PlaylistSelection() const {
        return mp3_ & MP3_PLAYLIST_SELECTION;
    }

    void setMp3PlaylistSelection(bool value) {
        if (value)
            mp3_ |= MP3_PLAYLIST_SELECTION;
        else
            mp3_ &= ~MP3_PLAYLIST_SELECTION;
    }
    
    //@}

    /** \name Radio Mode Register
     */
    //@{

    uint16_t radioFrequency() const {
        return (radio_ & RADIO_FREQUENCY) + RADIO_FREQUENCY_OFFSET;
    }

    void setRadioFrequency(uint16_t value) {
        radio_ = (radio_ & ~RADIO_FREQUENCY) | ((value - RADIO_FREQUENCY_OFFSET) & RADIO_FREQUENCY);
    }

    uint8_t radioStation() const {
        return (radio_ & RADIO_STATION) >> 9;
    }

    void setRadioStation(uint8_t id) {
        radio_ = (radio_ & ~RADIO_STATION) | ((id << 9) & RADIO_STATION);
    }

    bool radioManualTuning() const {
        return (radio_ & RADIO_MANUAL_TUNING);
    }

    void setRadioManualTuning(bool value) {
        if (value)
            radio_ |= RADIO_MANUAL_TUNING;
        else
            radio_ &= ~RADIO_MANUAL_TUNING;
    }

    //@}

    /** \name Temperature. 
     */
    //@{

    uint16_t temperatureKelvin() const {
        return temp_;
    }

    /** Returns the temperature in Celsius times 10. 
     */
    int16_t temperature() const {
        int16_t result = (temp_ / 4) - 4370;
        return result * 10 / 16;
    }

    void setTemperature(uint16_t kelvinx64) {
        temp_ = kelvinx64;
    }
    
    //}


private:

    friend class AVRPlayer;

    static constexpr uint8_t STATE_MODE = 7;
    static constexpr uint8_t STATE_DCDC_POWER = 1 << 3;
    static constexpr uint8_t STATE_VOL_BTN = 1 << 4;
    static constexpr uint8_t STATE_CTRL_BTN = 1 << 5;
    static constexpr uint8_t STATE_IDLE = 1 << 6;
    uint8_t state_ = 0;

    static constexpr uint8_t POWER_CHARGING = 1 << 7;
    static constexpr uint8_t POWER_VOLTAGE = 127;
    uint8_t power_ = 0;
    
    static constexpr uint16_t EVENT_ALARM = 1 << 0;
    static constexpr uint16_t EVENT_RTC_SYNC = 1 << 1;
    static constexpr uint16_t EVENT_VOL_PRESS = 1 << 2;
    static constexpr uint16_t EVENT_VOL_LONG_PRESS = 1 << 3;
    static constexpr uint16_t EVENT_CTRL_PRESS = 1 << 4;
    static constexpr uint16_t EVENT_CTRL_LONG_PRESS = 1 << 5;
    uint8_t events_ = 0;

    static constexpr uint8_t AUDIO_AUDIO_VOLUME = 15;
    static constexpr uint8_t AUDIO_AUDIO_SRC = 1 << 4;
    static constexpr uint8_t AUDIO_HEADPHONES = 1 << 5;
    static constexpr uint8_t AUDIO_LIGHTS = 1 << 6;
    uint8_t audio_ = 5; // start with volume 5 so that something can be heard immediately after start

    uint16_t control_ = 0;

    static constexpr uint16_t MP3_TRACK_ID = 1023;
    static constexpr uint16_t MP3_PLAYLIST_ID = 7 << 10;
    static constexpr uint16_t MP3_PLAYLIST_SELECTION = 1 << 15;
    uint16_t mp3_ = 0;

    static constexpr uint16_t RADIO_FREQUENCY = 511;
    static constexpr uint16_t RADIO_STATION = 7 << 9;
    static constexpr uint16_t RADIO_MANUAL_TUNING = 1 << 15;
    uint16_t radio_ = 0;

    uint16_t temp_ = 0;

} __attribute__((packed));


struct Clock {

    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t h;
    uint8_t m;
    uint8_t s;
    
    void secondTick() {
        if (++s < 60)
            return;
        s = 0;
        if (++m < 60)
            return;
        m = 0;
        if (++h < 24) 
            return;
        h = 0;
        if (++day <= DaysInMonth(month,year))
            return;
        day = 1;
        if (++month <= 12)
            return;
        month = 1;
        ++year;
    }

    static uint8_t DaysInMonth(uint8_t month, uint8_t year) {
      switch (month) {
        case 1: // Jan
        case 3: // Mar
        case 5: // May
        case 7: // Jul
        case 8: // Aug
        case 10: // Oct
        case 12: // Dec
            return 31;
        case 2 : // Feb
            // I'm ignoring the every 100 years leap year skip as the code will hopefully not be around for that long:)
            return (year % 4 == 0) ? 29 : 28;
        case 4:
        case 6:
        case 9:
        case 11:
        default: // whatever
            return 30;
      }
    }

    
} __attribute__((packed));


struct Command {

    struct SetMode {
        static constexpr uint8_t Id = 0;
        State::Mode mode;
    } __attribute__((packed));

    struct SetIdle {
        static constexpr uint8_t Id = 1;
        bool idle;
    } __attribute__((packed));

    struct SetVolume {
        static constexpr uint8_t Id = 2;
        uint8_t value;
    } __attribute__((packed));

    struct SetControl {
        static constexpr uint8_t Id = 3;
        uint16_t value;
        uint16_t maxValue;
    } __attribute__((packed));

    struct SetRadioState {
        static constexpr uint8_t Id = 4;
        uint8_t station;
        uint16_t frequency;
        bool manualTuning;

        SetRadioState(State const & state):
            station{state.radioStation()},
            frequency{state.radioFrequency()},
            manualTuning{state.radioManualTuning()} {
        }
        
    } __attribute__((packed));

    struct SetMP3State {
        static constexpr uint8_t Id = 5;
        uint8_t playlist;
        uint16_t track;
        bool playlistSelection;

        SetMP3State(State const & state):
            playlist{state.mp3PlaylistId()},
            track{state.mp3TrackId()},
            playlistSelection{state.mp3PlaylistSelection()} {
        }
        
    } __attribute__((packed));

    struct SetAudioLights {
        static constexpr uint8_t Id = 6;
        bool on;
    } __attribute__((packed));

    struct SetBrightness {
        static constexpr uint8_t Id = 7;
        uint8_t brightness;
    } __attribute__((packed));

    struct SetAccentColor {
        static constexpr uint8_t Id = 8;
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    } __attribute__((packed));

    struct SpecialLights {
        static constexpr uint8_t Id = 9;
        static constexpr uint8_t POINT = 0;
        static constexpr uint8_t BAR = 1;
        static constexpr uint8_t CENTERED_BAR = 2;
        uint8_t mode;
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        uint16_t value;
        uint16_t maxValue;
        uint16_t duration;
    } __attribute__((packed));

    struct Lights {
        static constexpr uint8_t Id = 10;
        uint8_t colors[24];
        uint16_t duration;
    } __attribute__((packed));
    
};

#endif // HAHA


