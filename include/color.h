#pragma once

class Color {
public:
    uint8_t g = 0;
    uint8_t r = 0;
    uint8_t b = 0;

    Color() = default;
    
    Color(uint8_t r, uint8_t g, uint8_t b):
        g{g},
        r{r},
        b{b} {
    }
};
