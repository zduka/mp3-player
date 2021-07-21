#pragma once

#include "state.h"

namespace msg {

    class Message {
    public:
        const uint8_t id;

    protected:
        Message(uint8_t id):
            id{id} {
        }
    } __attribute__((packed));

    template<typename T>
    static T const & At(uint8_t const * buffer) {
        return * pointer_cast<T const *>(buffer);
    }

    #ifdef ARCH_ESP8266
        template<typename T>
        static void Send(T const & msg) {
            Wire.beginTransmission(AVR_I2C_ADDRESS);
            unsigned char const * msgBytes = pointer_cast<unsigned char const *>(& msg);
            Wire.write(msgBytes, sizeof(T));
            Wire.endTransmission();
            /*
            Serial.print("I2C command sent:");
            for (unsigned i = 0; i < sizeof(T); ++i) {
                Serial.print(" ");
                Serial.print(msgBytes[i], HEX);
            }
            Serial.println("");
            */
        }
    #endif

    /** Turns the LEDs and ESP off and puts AVR to sleep. 
     */
    class PowerOff : public Message {
    public:
        static constexpr uint8_t Id = 0;

        PowerOff():
            Message{Id} {
        }

    private:

    } __attribute__((packed));


    /** Sets the mode and updates control & value ranges and colors. 
     */
    class SetMode : public Message {
    public:
        static constexpr uint8_t Id = 1;

        SetMode(State const & from):
            Message{Id},
            mode_{from.mode()}, 
            controlValues_{from.controlValues_},
            controlMaximums_{from.controlMaximums_},
            controlColor_{from.controlColor_},
            volumeColor_{from.volumeColor_} {
        }

        void applyTo(State & state) {
            state.setMode(mode_);
            state.controlValues_ = controlValues_;
            state.controlMaximums_ = controlMaximums_;
            state.controlColor_ = controlColor_;
            state.volumeColor_ = volumeColor_;
        }

    private:
        Mode mode_;
        uint16_t controlValues_;
        uint16_t controlMaximums_;
        Color controlColor_;
        Color volumeColor_;

    } __attribute__((packed)); // msg::SetMode

    class SetWiFiStatus : public Message {
    public:
        static constexpr uint8_t Id = 2;

        SetWiFiStatus(State const & from):
            Message{Id},
            status_{from.wifiStatus()} {
        }

        void applyTo(State & state) const {
            state.setWiFiStatus(status_);
        }

    private:
        WiFiStatus status_;
    } __attribute__((packed));

    static_assert(sizeof(SetWiFiStatus) == 2);

    class SetAudioSource : public Message {
    public:
        static constexpr uint8_t Id = 3;

        SetAudioSource(State const & from):
            Message{Id},
            source_{from.audioSource()} {
        }

        void applyTo(State & state) const {
            state.setAudioSource(source_);
        }

    private:
        AudioSource source_;

    } __attribute__((packed));

    
    class SetMP3Settings : public Message {
    public:
        static constexpr uint8_t Id = 4;

        SetMP3Settings(State const & from):
            Message{Id},
            raw_{from.mp3_} {
        }

        void applyTo(State & state) const {
            state.mp3_ = raw_;
        }

    private:
        uint16_t raw_;

    } __attribute__((packed));
    
    class SetRadioSettings : public Message {
    public:
        static constexpr uint8_t Id = 5;

        SetRadioSettings(State const & from):
            Message{Id},
            raw_{from.radio_} {
        }

        void applyTo(State & state) const {
            state.radio_ = raw_;
        }

    private:
        uint16_t raw_;

    } __attribute__((packed));

    class SetNightLightSettings : public Message {
    public:
        static constexpr uint8_t Id = 6;

        SetNightLightSettings(State const & from):
            Message{Id},
            raw_{from.nightLight_} {
        }

        void applyTo(State & state) const {
            state.nightLight_ = raw_;
        }

    private:
        uint16_t raw_;

    } __attribute__((packed));

    class SetAccentColor : public Message {
    public:
        static constexpr uint8_t Id = 7;

        SetAccentColor(State const & from):
            Message{Id},
            color_{from.accentColor()} {
        }

        void applyTo(State & state) const {
            state.setAccentColor(color_);
        }

    private:
        Color color_;

    }  __attribute__((packed));

    /** Instructs the ATTiny to display a bar of given parameters in the LED strip. 
     */
    class LightsBar : public Message {
    public:
        static constexpr uint8_t Id = 8;

        LightsBar(uint16_t value, uint16_t max, Color const & color, uint8_t timeout = 32):
            Message{Id},
            value{value},
            max{max},
            color{color},
            timeout{timeout} {
        }
        uint16_t value;
        uint16_t max;
        Color color;
        uint8_t timeout;
    } __attribute__((packed));



} // namespace msg

