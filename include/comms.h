#pragma once

class Message {
public:

    class SetControl;
    class SetVolume;

#ifdef ARCH_ESP8266
    static void Send(T const & msg) {
        Wire.beginTransmission(AVR_I2C_ADDRESS);
        Wire.write(T::Id);
        Wire.write(pointer_cast<char *>(& msg), sizeof(T));
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

    static SetControl const & At(uint8_t const * buffer) {
        return * pointer_cast<SetControl const *>(buffer);
    }
}; // Message::SetControl

class Message::SetVolume : public Message {
public:
    static constexpr uint8_t Id = 1;
    uint8_t value;
    uint8_t maxValue;
    
    static SetVolume const & At(uint8_t const * buffer) {
        return * pointer_cast<SetVolume const *>(buffer + 1);
    }
}; // Message::SetVolume
