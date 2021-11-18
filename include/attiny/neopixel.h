#pragma once

/** Library for working with neopixels. Comes with the megatinycore and does not require any additional arduino setup. 

    NOTE The library does not work well with 10MHz speed, 8MHz (or likely 16MHz) should be used instead. On 10MHz when turned completely off, the neopixels still gave faint light. The timing is actually really werid. Requires 8MHz clock debug info, *but* 10MHz actual clock.

 */
#include <tinyNeoPixel_Static.h>

#include "color.h"

template<uint8_t PIN, uint16_t SIZE>
class NeopixelStrip : public ColorStrip<SIZE> {
    using ColorStrip<SIZE>::colors_;
    using ColorStrip<SIZE>::changed_;
public:

    NeopixelStrip():
        leds_{SIZE, PIN, NEO_GRB + NEO_KHZ800, pointer_cast<uint8_t*>(& colors_)} {
        pinMode(PIN,OUTPUT);
    }

    void update() {
        if (changed_) {
            changed_ = false;
            leds_.show();
        }
    }
    
private:

    tinyNeoPixel leds_;
}; 
