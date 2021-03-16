#pragma once

#define AVR_I2C_ADDRESS 42


#define BUTTON_LONG_PRESS_TICKS 64
#define SPECIAL_LIGHTS_TIMEOUT 32
#define IRQ_MAX_DELAY 32

#define BUTTONS_COLOR Neopixel::Purple()
#define VOLUME_COLOR Neopixel::Blue()
#define CONTROL_COLOR Neopixel::Green()
#define AUDIO_COLOR accentColor_




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

struct Command {

    struct EnterMP3Mode {
        static constexpr uint8_t Id = 0;
        uint16_t controlValue;
        uint16_t controlMaxValue;

        EnterMP3Mode(uint16_t controlValue, uint16_t controlMaxValue):
            controlValue{controlValue},
            controlMaxValue{controlMaxValue} {
        }
    } __attribute__((packed));

    struct EnterRadioMode {
        static constexpr uint8_t Id = 1;
        uint16_t controlValue;
        uint16_t controlMaxValue;

        EnterRadioMode(uint16_t controlValue, uint16_t controlMaxValue):
            controlValue{controlValue},
            controlMaxValue{controlMaxValue} {
        }
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

    struct SetAudioLights {
        static constexpr uint8_t Id = 4;
        bool on;
    } __attribute__((packed));

    struct SetBrightness {
        static constexpr uint8_t Id = 5;
        uint8_t brightness;
    } __attribute__((packed));

    struct SetAccentColor {
        static constexpr uint8_t Id = 6;
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    } __attribute__((packed));

    struct SpecialLights {
        static constexpr uint8_t Id = 7;
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
        static constexpr uint8_t Id = 8;
        uint8_t colors[24];
        uint16_t duration;
    } __attribute__((packed));

    struct SetIdle {
        static constexpr uint8_t Id = 9;
        bool idle;
    } __attribute__((packed));

};

/** We can't use bitfields as sadly, they are implementation specific. 
 */
struct State {
public:
    enum class Mode : uint8_t {
        None = 0,                           
        MP3 = 1,
        Radio = 2,
        WWW = 3,
        NightLight = 4,
    };
    enum class AudioSrc : uint8_t {
        ESP = 0,
        Radio = 1,
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
    

    /** Shows the current input voltage. 

        Is constructed by reading the input voltage in scale 0..255 corresponding to 0..5v and then discarding the most signifficant byte as under normal conditions, this will always be 1. 

        127 == 5v from external source (and likely charging)
        86  == 4.2v, a fully charged li-ion battery
        60  == 3.7v, nominal value for 
        40  == 3.3v, value at which the low battery warning is given
        25  == 3v, value at which attiny goes immediately to deepsleep again
    */
    uint16_t voltage() const {
        return 500 * ((power_ & POWER_VOLTAGE) + 128) / 255;
    }

    bool charging() const {
        return power_ & POWER_CHARGING;
    }

    bool irq() const {
        return events_ & EVENT_IRQ;
    }

    void setIrq() {
        events_ |= EVENT_IRQ;
    }

    void clearEvents() {
        events_ = 0;
    }

    bool alarm() const {
        return events_ & EVENT_ALARM;
    }
    
    bool rtcSync() const {
        return events_ & EVENT_RTC_SYNC;
    }
    
    bool volumeChange() const {
        return events_ & EVENT_VOL_CHANGE;
    }
    
    bool controlChange() const {
        return events_ & EVENT_CTRL_CHANGE;
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
    
    bool lowBatteryChanged() const {
        return events_ & EVENT_LOW_BATTERY_CHANGE;
    }
    
    bool headphonesChanged() const {
        return events_ & EVENT_HEADPHONES_CHANGE;
    }

    /** \name Audio Register.
     */
    //@{
    uint8_t volume() const {
        return audio_ & AUDIO_AUDIO_VOLUME;
    }

    void setVolume(uint8_t value, bool recordChange = true) {
        audio_ = (audio_ & ~AUDIO_AUDIO_VOLUME) + (value & AUDIO_AUDIO_VOLUME);
        if (recordChange)
            events_ |= EVENT_VOL_CHANGE;
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

    uint16_t control() const {
        return control_;
    }

    void setControl(uint16_t value, bool recordChange = true) {
        control_ = value;
        if (recordChange)
            events_ |= EVENT_CTRL_CHANGE;
    }

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
    
    static constexpr uint16_t EVENT_IRQ = 1 << 0;
    static constexpr uint16_t EVENT_ALARM = 1 << 1;
    static constexpr uint16_t EVENT_RTC_SYNC = 1 << 2;
    static constexpr uint16_t EVENT_VOL_CHANGE = 1 << 3;
    static constexpr uint16_t EVENT_CTRL_CHANGE = 1 << 4;
    static constexpr uint16_t EVENT_VOL_PRESS = 1 << 5;
    static constexpr uint16_t EVENT_VOL_LONG_PRESS = 1 << 6;
    static constexpr uint16_t EVENT_CTRL_PRESS = 1 << 7;
    static constexpr uint16_t EVENT_CTRL_LONG_PRESS = 1 << 8;
    static constexpr uint16_t EVENT_LOW_BATTERY_CHANGE = 1 << 9;
    static constexpr uint16_t EVENT_HEADPHONES_CHANGE = 1 << 10;
    uint16_t events_ = 0;


    static constexpr uint8_t AUDIO_AUDIO_VOLUME = 15;
    static constexpr uint8_t AUDIO_AUDIO_SRC = 1 << 4;
    static constexpr uint8_t HEADPHONES = 1 << 5;
    static constexpr uint8_t AUDIO_LIGHTS = 1 << 6;
    uint8_t audio_ = 0;

    uint16_t control_ = 0;
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

#ifdef HAHA

/** The basic state
 */
struct AVRState {

    /** \name Interrupt and its causes.

        These are cleared automatically once the AVRSTate is read. 
     */
    //@{
    unsigned irq : 1; // notifies esp8266 to talk to avr
    unsigned alarm : 1; // when high, an alarm has triggered the poweron
    unsigned rtcSync : 1; // esp should silently check alarms, updates, clock, etc.
    unsigned volChange : 1; // volume has changed (encoder)
    unsigned ctrlChange : 1; // control value has changed (encoder)
    unsigned volPress : 1; // volume button was pressed
    unsigned volLongPress : 1; // volume button was long-pressed
    unsigned ctrlPress : 1; // control button was pressed

    unsigned ctrlLongPress : 1; // control button was long-pressed
    unsigned chargingChange : 1; // charging has been enabled or disabled
    unsigned lowBatteryChange : 1; // low battery has been enabled or disabled
    unsigned headphonesChange : 1; // headphones has been plugged or unplugged
    //@}
    
    /** \name Power
     */
    //@{
    unsigned dcdcPower : 1; // when high, 3v3 and 5v rails are on
    unsigned charging : 1; // when high, device is charging
    unsigned voltage : 7;
    //@}

    /** \name Controls

        Holds the actual values for the controls. 
     */
    //@{
    unsigned volBtn : 1; // 1 if pressed
    unsigned ctrlBtn : 1; // 1 if pressed
    /** This is always current volume. 

        16 steps are supported at most.
     */
    unsigned volume : 4;
    /** Value of the control encoder. 

        As this can also encode the actual radio frequency, it needs to be reasonably big. 
     */
    unsigned control : 11; // 0..2047   
    //@}

    /** Audio settings. 
     */
    //@{
    unsigned headphones : 1; // 0 = not connected, 1 = connected
    unsigned audioSource : 1; // 0 = mp3 (esp), 1 = fm module
    unsigned mode : 3; // at most 8 modes are supported
    
    //@}

} __attribute__((packed));

#endif 

