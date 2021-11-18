#pragma once

#define DEFAULT_AP_SSID "mp3-player"
#define DEFAULT_AP_PASSWORD "mp3-player"

/** Timeout of the player depending on whether the player is playing, or idle. In minutes, values from 1 to 255 are supported. 
 */
#define DEFAULT_IDLE_TIMEOUT 30
#define DEFAULT_PLAY_TIMEOUT 60

#define AVR_I2C_ADDRESS 67
#define IRQ_RESPONSE_TIMEOUT 64

#define USB_VOLTAGE_THRESHOLD 440

/** When the battery level drops below this threshold, the low battery flag is raised.
 */
#define BATTERY_LOW_VCC 350

/** When battery reaches this voltage, the critical alert (three red flashes) will be displayed and the player will go to sleep immediately. 
 */
#define BATTERY_CRITICAL_VCC 340

/** When battery reaches this level, no warnings are displayed even and the sleep is immediately resumed. 
 */
#define BATTERY_DEAD_VCC 320

/** The battery undervoltage (critical or dead) must be detected consecutively for the specified number of measurements for the action to be taken to prevent the player reacting to quick power fluctuations. 
 
    Upon wakeup, the AVR waits the given ammount of time for the measurements to make sure ESP is not powered up in vain. 
 */
#define UNDERVOLTAGE_TIMEOUT 20

#define ESP_BUSY_TIMEOUT 64 * 60 * 1 // 1 minute timeout for busy

#define DEFAULT_VOLUME 5
#define DEFAULT_ALARM_VOLUME 5
#define MAX_VOLUME 16

#define MAX_WALKIE_TALKIE_MESSAGES 8

/** A walkie talkie recording shorter than this number of milliseconds will be ignored. 
 */
#define MIN_WALKIE_TALKIE_RECORDING 1000 * 1

/** If the walkie-talkie recording will be longer than this number of milliseconds, it will be automatically stopped and sent.
 */
#define MAX_WALKIE_TALKIE_RECORDING 1000 * 10

/** Timeout for connection in the sync mode in milliseconds. 
 */
#define SYNC_CONNECTION_TIMEOUT (120 * 1000)

/** Number of ticks (1/64th of a second) for which a button must be pressed down uninterrupted to power the player on. 
 
    Currently set to 2 seconds
 */
#define POWER_ON_PRESS_TICKS 128
#define BUTTON_LONG_PRESS_TICKS 96
#define BUTTON_LONG_PRESS_THRESHOLD 84

#define SPECIAL_LIGHTS_TIMEOUT 128





#define DEFAULT_COLOR Color::White()
#define DEFAULT_BRIGHTNESS 32

/** Colors of the available modes. 
 */
#define MODE_COLOR_MP3 Color::Blue()
#define MODE_COLOR_RADIO Color::Green()
#define MODE_COLOR_WALKIE_TALKIE Color::Cyan()
#define MODE_COLOR_LIGHTS Color::Yellow()

/** Colors for the binary clock idle mode. 
 */
#define BINARY_CLOCK_HOURS Color::Blue()
#define BINARY_CLOCK_MINUTES Color::Green()
#define BINARY_CLOCK_BATTERY Color::Red()
#define BINARY_CLOCK_CHARGING Color::Cyan()

/** Color of the double long press timeout bar. 
 */
#define DOUBLE_LONG_PRESS_COLOR Color::Red()


/** Radio frequency min and max values (in Mhz x 10). This must be within the chip's usable range and currently stands at 76.0 - 108.0 Mhz
 */
#define RADIO_FREQUENCY_MIN 760
#define RADIO_FREQUENCY_MAX 1080
