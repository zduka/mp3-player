#pragma once

class Color {
public:
    uint8_t g = 0;
    uint8_t r = 0;
    uint8_t b = 0;

    Color() = default;

    Color(Color const &) = default;

    Color(Color const volatile & from):
        g{from.g},
        r{from.r},
        b{from.b} {
    }            

    Color(uint8_t r, uint8_t g, uint8_t b):
        g{g},
        r{r},
        b{b} {
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
