#pragma once

/** Library for working with neopixels. Comes with the megatinycore and does not require any additional arduino setup. 

    NOTE The library does not work well with 10MHz speed, 8MHz (or likely 16MHz) should be used instead. On 10MHz when turned completely off, the neopixels still gave faint light. 
 */
#include <tinyNeoPixel_Static.h>

#include "color.h"

template<uint8_t PIN, uint16_t SIZE>
class NeopixelStrip {
public:

    NeopixelStrip():
        leds_{SIZE, PIN, NEO_GRB + NEO_KHZ800, pointer_cast<uint8_t*>(& current_)} {
        pinMode(PIN,OUTPUT);
    }

    void setAll(Color const & color) {
        for (uint8_t i = 0; i < SIZE; ++i) {
            target_[i] = color;
        }
        updated_ = true;
    }

    void addColor(uint8_t index, Color const & color) {
        target_[index].add(color);
        updated_ = true;
    }

    /** Shows point at given offset. 
     */
    void showPoint(uint16_t value, uint16_t max, Color const & color) {
        uint8_t v = 255;
        uint32_t offset = static_cast<uint32_t>(value) * (SIZE - 1) * 255 / max;
        for (uint8_t i = 0; i < SIZE; ++i) {
            uint8_t b = 0;
            // if offset is larger than pixel, stay black and remove from offset
            if (offset >= 255) {
                offset -= 255;
            // otherwise if offset is 0, just use value    
            } else if (offset == 0) {
                b = v;
                v = 0;
                // otherwise we have some offset and some value, and the value has to be split
            } else {
                b = 255 - (offset & 0xff);
                v -= b; // we know it
                offset = 0;
            }
            target_[i] = color.withBrightness(b);
        }
        updated_ = true;
    }

    /** Shows bar from the beginning to the given value. 
     */
    void showBar(uint16_t value, uint16_t max, Color const & color) {
        uint32_t v = static_cast<uint32_t>(value) * (SIZE * 255) / max;
        for (uint8_t i = 0; i < SIZE; ++i) {
            uint8_t b = v > 255 ? 255 : (v & 0xff);
            v -= b;
            target_[i] = color.withBrightness(b);
        }
        updated_ = true;
    }

    /** Shows a bar at given value that grows symmetrically from the center. 
     */
    void showCenteredBar(uint16_t value, uint16_t max, Color const & color) {
        uint32_t v = static_cast<uint32_t>(value) * (SIZE * 255) / max;
        uint32_t offset = (SIZE * 255 - v) / 2;
        for (uint8_t i = 0; i < SIZE; ++i) {
            uint8_t b = (offset > 255) ? 0 : (255 - offset);
            offset -= (255 - b);
            if (v < b)
                b = v;
            v -= b;
            target_[i] = color.withBrightness(b);
        }
        updated_ = true;
    }

    void tick(uint8_t step = 16) {
        if (! updated_)
            return;
        updated_ = false;
        for (uint8_t i = 0; i < SIZE; ++i)
            updated_ = current_[i].moveTowards(target_[i], step) || updated_;
    }

    /** Identical to tick, but reverses the order of the visible pixels. 
     */
    void reversedTick(uint8_t step = 16) {
        if (! updated_)
            return;
        updated_ = false;
        for (uint8_t i = 0; i < SIZE; ++i)
            updated_ = current_[i].moveTowards(target_[SIZE - 1 - i], step) || updated_;
    }

    void update(bool force = false) {
        if (updated_ || force) {
            updated_ = false;
            leds_.show();
        }
    }

    void sync() {
        for (uint8_t i = 0; i < SIZE; ++i)
            current_[i] = target_[i];
    }

    void reversedSync() {
        for (uint8_t i = 0; i < SIZE; ++i)
            current_[i] = target_[SIZE - 1 - i];
    }
    
    Color & operator[](unsigned index) {
        return target_[index];
        updated_ = true;
    }

private:

    Color current_[SIZE];
    Color target_[SIZE];
    volatile bool updated_ = false;
    tinyNeoPixel leds_;
}; 
