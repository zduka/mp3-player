#pragma once

#if (defined ARCH_ESP8266)

#define LOG(...) Log_(String("") + __VA_ARGS__)
#define STR(...) (String("") + __VA_ARGS__)

inline void Log_(String const & str) {
    Serial.print(String(millis() / 1000) + ": ");
    Serial.println(str);
}

#endif


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
