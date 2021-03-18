#pragma once

/** Library for working with neopixels. Comes with the megatinycore and does not require any additional arduino setup. 

    NOTE The library does not work well with 10MHz speed, 8MHz (or likely 16MHz) should be used instead. On 10MHz when turned completely off, the neopixels still gave faint light. 
 */
#include <tinyNeoPixel_Static.h>


/** Single neopixel. 
 */
class Neopixel {
public:
    uint8_t g = 0;
    uint8_t r = 0;
    uint8_t b = 0;

    Neopixel() = default;

    Neopixel(uint8_t r, uint8_t g, uint8_t b):
        g{g},
        r{r},
        b{b} {
    }

    bool isBlack() const {
        return g == 0 && r == 0 && b == 0;
    }

    bool operator == (Neopixel const & other) const {
        return g == other.g && r == other.r && b == other.b;
    }

    bool moveTowards(Neopixel const & target, uint8_t step = 16) {
        bool result = MoveChannelTowards(g, target.g, step);
        result = MoveChannelTowards(r, target.r, step) || result;
        result = MoveChannelTowards(b, target.b, step) || result;
        return result;
    }

    Neopixel withBrightness(uint8_t brightness) const {
        switch (brightness) {
            case 255:
                return *this;
            case 0:
                return Neopixel{};
            default: {
                uint8_t rr = r * brightness / 255;
                uint8_t gg = g * brightness / 255;
                uint8_t bb = b * brightness / 255;
                return Neopixel{rr,gg,bb};
            }
        }
    }

    static Neopixel Black() { return Neopixel{0,0,0}; }
    static Neopixel White() { return Neopixel{255,255,255}; }
    static Neopixel Green() { return Neopixel{0,255, 0}; }
    static Neopixel Blue() { return Neopixel{0,0,255}; }
    static Neopixel Red() { return Neopixel{255,0,0}; }
    static Neopixel Purple() { return Neopixel{255,0,255}; }

    static Neopixel DarkPurple() { return Neopixel{128, 0, 128}; }

private:
    static bool MoveChannelTowards(uint8_t & channel, uint8_t target, uint8_t step) {
        if (channel == target)
            return false;
        if (abs(channel - target) <= step)
            channel = target;
        else if (channel < target)
            channel += step;
        else
            channel -= step;
        return true;
    }
    
    
} __attribute__((packed));


template<uint8_t PIN, uint16_t SIZE>
class NeopixelStrip {
public:

    NeopixelStrip():
        leds_{SIZE, PIN, NEO_GRB + NEO_KHZ800, pointer_cast<uint8_t*>(& current_)} {
        pinMode(PIN,OUTPUT);
    }

    void setAll(Neopixel const & color) {
        for (uint8_t i = 0; i < SIZE; ++i) {
            target_[i] = color;
        }
        updated_ = true;
    }

    /** Shows point at given offset. 
     */
    void showPoint(uint16_t value, uint16_t max, Neopixel const & color) {
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
    void showBar(uint16_t value, uint16_t max, Neopixel const & color) {
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
    void showCenteredBar(uint16_t value, uint16_t max, Neopixel const & color) {
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
        if (updated_) {
            updated_ = false;
            for (uint8_t i = 0; i < SIZE; ++i)
                updated_ = current_[i].moveTowards(target_[i], step) || updated_;
            if (updated_)
                update();
        }
    }

    void update() {
        leds_.show();
    }

    void sync() {
        if (updated_) {
            for (uint8_t i = 0; i < SIZE; ++i)
                current_[i] = target_[i];
            updated_ = false;
            update();
        }
    }
    
    Neopixel & operator[](unsigned index) {
        return target_[index];
        updated_ = true;
    }

private:

    Neopixel current_[SIZE];
    Neopixel target_[SIZE];
    volatile bool updated_ = false;
    tinyNeoPixel leds_;
}; 
