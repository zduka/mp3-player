#pragma once

#define AVR_I2C_ADDRESS 67
#define IRQ_RESPONSE_TIMEOUT 64

#define BATTERY_CRITICAL_VCC 340

#define DEFAULT_VOLUME 5


/** Number of ticks (1/64th of a second) for which a button must be pressed down uninterrupted to power the player on. 
 
    Currently set to 2 seconds
 */
#define POWER_ON_PRESS_TICKS 128
#define BUTTON_LONG_PRESS_TICKS 128
#define BUTTON_LONG_PRESS_THRESHOLD 96

#define SPECIAL_LIGHTS_TIMEOUT 128




#define DEFAULT_COLOR Color::White()
#define DEFAULT_BRIGHTNESS 32

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
