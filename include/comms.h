#pragma once

class Message {
public:

    class SetControl;
    class SetVolume;

#ifdef ARCH_ESP8266
    template<typename T>
    static void Send(T const & msg) {
        Wire.beginTransmission(AVR_I2C_ADDRESS);
        Wire.write(T::Id);
        Wire.write(pointer_cast<char const *>(& msg), sizeof(T));
        Wire.endTransmission();
    }
#endif

protected:
    
}; // class Message

class Message::SetControl : public Message {
public:
    static constexpr uint8_t Id = 0;
    uint16_t value;
    uint16_t maxValue;

    SetControl(uint16_t value, uint16_t maxValue):
        value{value},
        maxValue{maxValue} {
    }

    static SetControl const & At(uint8_t const * buffer) {
        return * pointer_cast<SetControl const *>(buffer);
    }
} __attribute__((packed)); // Message::SetControl

static_assert(sizeof(Message::SetControl) == 4);

class Message::SetVolume : public Message {
public:
    static constexpr uint8_t Id = 1;
    uint8_t value;
    uint8_t maxValue;

    SetVolume(uint8_t value, uint8_t maxValue):
        value{value},
        maxValue{maxValue} {
    }

    static SetVolume const & At(uint8_t const * buffer) {
        return * pointer_cast<SetVolume const *>(buffer + 1);
    }
} __attribute__((packed)); // Message::SetVolume

static_assert(sizeof(Message::SetVolume) == 2);
