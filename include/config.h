#pragma once

#define AVR_I2C_ADDRESS 67

#define SPECIAL_LIGHTS_TIMEOUT 128


#define DEFAULT_COLOR Color::White()

/** Colors of the available modes. 
 */
#define MODE_COLOR_MP3 Color::Blue()
#define MODE_COLOR_RADIO Color::Green()
#define MODE_COLOR_WALKIE_TALKIE Color::Cyan()
#define MODE_COLOR_NIGHT_LIGHT Color::Yellow()

/** Color of the double long press timeout bar. 
 */
#define DOUBLE_LONG_PRESS_COLOR Color::Red()




/** Radio frequency min and max values (in Mhz x 10). This must be within the chip's usable range and currently stands at 76.0 - 108.0 Mhz
 */
#define RADIO_FREQUENCY_MIN 760
#define RADIO_FREQUENCY_MAX 1080
