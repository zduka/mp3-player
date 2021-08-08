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


#if (defined ARCH_ESP8266)

class Log {
public:
    static constexpr size_t BufferSize = 256;
    static inline char Buffer[BufferSize];

    static void Print(char const * format, ...) { \
        va_list args; \
        va_start(args, format); \
        snprintf_P(Buffer, BufferSize, format, args); \
        va_end(args); \        
        Serial.print(millis() / 1000); \
        Serial.print(PSTR(": ")); \
        Serial.write(pointer_cast<char*>(& Log::Buffer)); \
        Serial.println(); \
    }
}; // Log


#define LOGF(FORMAT,...) Log::Print(PSTR(FORMAT) __VA_OPT__(,) __VA_ARGS__)

/*
#define LOGF(FMT, ...) do { \
    snprintf_P(Log::Buffer, Log::BufferSize, PSTR(FMT), __VA_ARGS__); \
    Serial.print(millis() / 1000); \
    Serial.print(PSTR(": ")); \
    Serial.write(pointer_cast<char*>(& Log::Buffer)); Serial.println(); \
} while (false)
*/

#define LOG(...) Log_(String("") + __VA_ARGS__)
#define STR(...) (String("") + __VA_ARGS__)

inline void Log_(String const & str) {
    Serial.print(String(millis() / 1000) + ": ");
    Serial.println(str);
}

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