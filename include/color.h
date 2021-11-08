#pragma once

#include "helpers.h"

class Color {
public:
    uint8_t g;
    uint8_t r;
    uint8_t b;

    /** Creates new color. 
     
        Does not initialize the color to any particular value so that the constructor is trivial and color can be part of anonymous structs & unions.
     */
    Color() = default;

    Color(Color const &) = default;

    Color(Color const volatile & from):
        g{from.g},
        r{from.r},
        b{from.b} {
    }     

    static Color HTML(char const * str) {
        if (*str == '#') // optional #
            ++str;
        uint8_t r = FromHex(str[0]) * 16 + FromHex(str[1]);
        uint8_t g = FromHex(str[2]) * 16 + FromHex(str[3]);
        uint8_t b = FromHex(str[4]) * 16 + FromHex(str[5]);
        return Color{r, g, b};
    }

    static Color RGB(uint8_t r, uint8_t g, uint8_t b) {
        return Color{r, g, b};
    }        

    /** Creates color based on the HSV model coordinates. 
     
        The code is straight from Adafruit Neopixel library.
     */
    static Color HSV(uint16_t h, uint8_t s, uint8_t v) {
        uint8_t red, green, blue;
        // Remap 0-65535 to 0-1529. Pure red is CENTERED on the 64K rollover;
        // 0 is not the start of pure red, but the midpoint...a few values above
        // zero and a few below 65536 all yield pure red (similarly, 32768 is the
        // midpoint, not start, of pure cyan). The 8-bit RGB hexcone (256 values
        // each for red, green, blue) really only allows for 1530 distinct hues
        // (not 1536, more on that below), but the full unsigned 16-bit type was
        // chosen for hue so that one's code can easily handle a contiguous color
        // wheel by allowing hue to roll over in either direction.
        h = (h * 1530L + 32768) / 65536;
        // Because red is centered on the rollover point (the +32768 above,
        // essentially a fixed-point +0.5), the above actually yields 0 to 1530,
        // where 0 and 1530 would yield the same thing. Rather than apply a
        // costly modulo operator, 1530 is handled as a special case below.

        // So you'd think that the color "hexcone" (the thing that ramps from
        // pure red, to pure yellow, to pure green and so forth back to red,
        // yielding six slices), and with each color component having 256
        // possible values (0-255), might have 1536 possible items (6*256),
        // but in reality there's 1530. This is because the last element in
        // each 256-element slice is equal to the first element of the next
        // slice, and keeping those in there this would create small
        // discontinuities in the color wheel. So the last element of each
        // slice is dropped...we regard only elements 0-254, with item 255
        // being picked up as element 0 of the next slice. Like this:
        // Red to not-quite-pure-yellow is:        255,   0, 0 to 255, 254,   0
        // Pure yellow to not-quite-pure-green is: 255, 255, 0 to   1, 255,   0
        // Pure green to not-quite-pure-cyan is:     0, 255, 0 to   0, 255, 254
        // and so forth. Hence, 1530 distinct hues (0 to 1529), and hence why
        // the constants below are not the multiples of 256 you might expect.

        // Convert hue to R,G,B (nested ifs faster than divide+mod+switch):
        if (h < 510) {         // Red to Green-1
            blue = 0;
            if(h < 255) {       //   Red to Yellow-1
                red = 255;
                green = h;            //     g = 0 to 254
            } else {              //   Yellow to Green-1
                red = 510 - h;      //     r = 255 to 1
                green = 255;
            }
        } else if (h < 1020) { // Green to Blue-1
            red = 0;
            if (h <  765) {      //   Green to Cyan-1
                green = 255;
                blue = h - 510;      //     b = 0 to 254
            } else {              //   Cyan to Blue-1
                green = 1020 - h;     //     g = 255 to 1
                blue = 255;
            }
        } else if(h < 1530) { // Blue to Red-1
            green = 0;
            if (h < 1275) {      //   Blue to Magenta-1
                red = h - 1020;     //     r = 0 to 254
                blue = 255;
            } else {              //   Magenta to Red-1
                red = 255;
                blue = 1530 - h;     //     b = 255 to 1
            }
        } else {                // Last 0.5 Red (quicker than % operator)
            red = 255;
            green = blue = 0;
        }

        // Apply saturation and value to R,G,B, pack into 32-bit result:
        uint32_t v1 =   1 + v; // 1 to 256; allows >>8 instead of /255
        uint16_t s1 =   1 + s; // 1 to 256; same reason
        uint8_t  s2 = 255 - s; // 255 to 0
        red = ((((red * s1) >> 8) + s2) * v1) >> 8;
        green = ((((green * s1) >> 8) + s2) * v1) >> 8;
        blue = ((((blue * s1) >> 8) + s2) * v1) >> 8;
        return Color{red, green, blue};
    }

    Color & operator = (Color const &) = default;

    void operator = (Color const & from) volatile {
        g = from.g;
        r = from.r;
        b = from.b;
    }

    bool isBlack() const {
        return g == 0 && r == 0 && b == 0;
    }

    bool operator == (Color const & other) const {
        return g == other.g && r == other.r && b == other.b;
    }

    bool operator != (Color const & other) const {
        return g != other.g || r != other.r || b != other.b;
    }

    bool moveTowards(Color const & target, uint8_t step = 16) {
        bool result = MoveChannelTowards(g, target.g, step);
        result = MoveChannelTowards(r, target.r, step) || result;
        result = MoveChannelTowards(b, target.b, step) || result;
        return result;
    }

    void add(Color const & color) {
        r = (255 - color.r < r) ? 255 : r + color.r;
        g = (255 - color.g < g) ? 255 : g + color.g;
        b = (255 - color.b < b) ? 255 : b + color.b;
    }

    Color withBrightness(uint8_t brightness) const {
        switch (brightness) {
            case 255:
                return *this;
            case 0:
                return Color{};
            default: {
                uint8_t rr = r * brightness / 255;
                uint8_t gg = g * brightness / 255;
                uint8_t bb = b * brightness / 255;
                return Color{rr,gg,bb};
            }
        }
    }

    static Color Black() { return Color{0,0,0}; }
    static Color White() { return Color{255,255,255}; }
    static Color Red() { return Color{255,0,0}; }
    static Color Green() { return Color{0,255, 0}; }
    static Color Blue() { return Color{0,0,255}; }
    static Color Purple() { return Color{255,0,255}; }
    static Color Yellow() { return Color{255,255,0}; }
    static Color Cyan() { return Color{0,255,255}; }

    static Color DarkRed() { return Color{128, 0, 0}; }
    static Color DarkPurple() { return Color{128, 0, 128}; }

private:

    Color(uint8_t r, uint8_t g, uint8_t b):
        g{g},
        r{r},
        b{b} {
    }

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

/** Array of N pixels that supports basic drawing and effects. 
 */
template<uint16_t SIZE>
class ColorStrip {
public:
    Color & operator[](unsigned index) {
        changed_ = true;
        return colors_[index];
    }

    void fill(Color const & color, uint8_t step = 255) {
        for (uint8_t i = 0; i < SIZE; ++i) {
            changed_ = colors_[i].moveTowards(color, step) | changed_;
        }
    }

    void withBrightness(uint8_t brightness) {
        for (uint8_t i = 0; i < SIZE; ++i) {
            Color c = colors_[i].withBrightness(brightness);
            if (c != colors_[i]) {
                colors_[i] = c;
                changed_ = true; 
            }
        }
    }

    void moveTowards(ColorStrip<SIZE> const & other, uint8_t step) {
        for (uint8_t i = 0; i < SIZE; ++i) {
            changed_ = colors_[i].moveTowards(other.colors_[i], step) | changed_;
        }
    }

    void moveTowardsReversed(ColorStrip<SIZE> const & other, uint8_t step) {
        for (uint8_t i = 0; i < SIZE; ++i) {
            changed_ = colors_[i].moveTowards(other.colors_[SIZE - 1 - i], step) | changed_;
        }
    }

    /** Shows point at given offset. 
     */
    void showPoint(uint16_t value, uint16_t max, Color const & color, uint8_t step = 255) {
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
            changed_ = colors_[i].moveTowards(color.withBrightness(b), step) | changed_;
        }
    }

    /** Shows bar from the beginning to the given value. 
     */
    void showBar(uint16_t value, uint16_t max, Color const & color, uint8_t step = 255) {
        uint32_t v = static_cast<uint32_t>(value) * (SIZE * 255) / max;
        for (uint8_t i = 0; i < SIZE; ++i) {
            uint8_t b = v > 255 ? 255 : (v & 0xff);
            v -= b;
            changed_ = colors_[i].moveTowards(color.withBrightness(b), step) | changed_;
        }
    }

    /** Shows a bar at given value that grows symmetrically from the center. 
     */
    void showBarCentered(uint16_t value, uint16_t max, Color const & color, uint8_t step = 255) {
        if (max < value)
            max = value;
        uint32_t v = static_cast<uint32_t>(value) * (SIZE * 255) / max;
        uint32_t offset = (SIZE * 255 - v) / 2;
        for (uint8_t i = 0; i < SIZE; ++i) {
            uint8_t b = (offset > 255) ? 0 : (255 - offset);
            offset -= (255 - b);
            if (v < b)
                b = v;
            v -= b;
            changed_ = colors_[i].moveTowards(color.withBrightness(b), step) | changed_;
        }
    }

    void markAsChanged() {
        changed_ = true;
    }

protected:
    Color colors_[SIZE];
    bool changed_ = false;
}; // ColorStrip