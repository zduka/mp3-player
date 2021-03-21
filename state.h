#pragma once

#define AVR_I2C_ADDRESS 42


#define ESP_VOLUME_STEP 0.2

#define BUTTON_LONG_PRESS_TICKS 64
#define SPECIAL_LIGHTS_TIMEOUT 32
#define IRQ_MAX_DELAY 32
#define RADIO_FREQUENCY_OFFSET 760
#define RADIO_FREQUENCY_MAX 320

#define BUTTONS_COLOR Neopixel::DarkRed()
#define BUTTONS_LONG_PRESS_COLOR Neopixel::Red()
#define VOLUME_COLOR Neopixel::Yellow()
#define RADIO_STATION_COLOR Neopixel::Green()
#define RADIO_FREQUENCY_COLOR Neopixel::Cyan()
#define MP3_TRACK_COLOR Neopixel::Blue()
#define MP3_PLAYLIST_COLOR Neopixel::Purple()
#define AUDIO_COLOR accentColor_

/** The values of resistors forming the voltage divider from up to 5V to the reference voltage of 1.1V at the VCC_SENSE pin. For better accuracy.
 */
#define VCC_DIVIDER_RES1 386
#define VCC_DIVIDER_RES2 100

/** \name Pointer-to-pointer cast
 
    Since any pointer to pointer cast can be done by two static_casts to and from `void *`, this simple template provides a shorthand for that functionality.
 */
//@{
template<typename T, typename W>
inline T pointer_cast(W const * from) {
    return static_cast<T>(static_cast<void const *>(from));
}

template<typename T, typename W>
inline T pointer_cast(W * from) {
    return static_cast<T>(static_cast<void *>(from));
}
//@}


/** We can't use bitfields as sadly, they are implementation specific. 

    The state is kept at AVR, which preserves it throughout power cycles as long as the battery is not completely depleted. 
 */
struct State {
public:

    /** \name State register.
     */
    //@{
    enum class Mode : uint8_t {
        MP3 = 0,
        Radio = 1,
        WWW = 2,
        NightLight = 3,
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

        VCC voltage times 100. Values from 340 to 500 are expected. Internally, the state stores the ADC result. 
 
    */
    uint16_t voltage() const {
        uint16_t result = (power_ & POWER_VOLTAGE);
        return (result == 0) ? 0 : (result * 160 / 127 + 340);
    }

    void setVoltage(uint16_t value) {
        value = (value > 500) ? 500 : value;
        value = (value > 340) ? (value - 340) : value;
        value = (value * 127) / 160;
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
    static constexpr uint8_t HEADPHONES = 1 << 5;
    static constexpr uint8_t AUDIO_LIGHTS = 1 << 6;
    uint8_t audio_ = 0;

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


