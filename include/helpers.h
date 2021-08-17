#pragma once

/** \name Pointer-to-pointer cast
 
    Since any pointer to pointer cast can be done by two static_casts to and from `void *`, this simple template provides a shorthand for that functionality.
 */
//@{
template<typename T, typename W>
inline T pointer_cast(W const * from) {
    return static_cast<T>(static_cast<void const *>(from));
}

template<typename T, typename W>
inline T pointer_cast(W volatile * from) {
    return static_cast<T>(static_cast<void *>(const_cast<W*>(from)));
}

template<typename T, typename W>
inline T pointer_cast(W * from) {
    return static_cast<T>(static_cast<void *>(from));
}
//@}

#define assert(...)


#if (defined ARCH_ESP8266)


// TODO __VA_OPT__(,) works better than the GNU extension here with C++20, but although it works for ESP, it confuses the editor so I am stuck with ##__VA_ARGS__ for now
#define LOG(FORMAT,...) do { Serial.printf_P(PSTR("%u: "), millis() / 1000); Serial.printf_P(PSTR(FORMAT), ##__VA_ARGS__); Serial.println(); } while (false)

#define STR(...) (String("") + __VA_ARGS__)

#endif



inline uint8_t FromHex(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    else if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    else if (c>= 'a' && c <= 'f')
        return c - 'a' + 10;
    else
        return 0;
}

inline char ToHex(uint8_t value) {
    if (value < 10)
        return '0' + value;
    else if (value < 16)
        return 'a' + value - 10;
    else 
        return '?'; // error
}

// because WiFiClientSecure does not do single character write:(
template<typename STREAM>
void WriteByte(STREAM & s, uint8_t byte) {
    s.write(& byte, 1);
}

template<typename STREAM>
void UrlEncode(STREAM & s, char const * what) {
    while (*what != 0) {
        if (*what == ' ') {
            WriteByte(s, '+');
        } else if (isalnum(*what)) {
            WriteByte(s,*what);
        } else {
            WriteByte(s, '%');
            WriteByte(s, ToHex(*what >> 4));
            WriteByte(s, ToHex(*what && 0xf));
        }
        ++what;
    }
}
