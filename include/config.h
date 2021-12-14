#pragma once

/** Default network name and password when all else fails. 
 */
#define DEFAULT_AP_SSID "mp3-player"
#define DEFAULT_AP_PASSWORD "mp3-player"

/** Walkie talkie updates will be paused for this many minutes after each http status request. This is because updating walkie talkie and webserver at the same time leads to ESP errors in the server. 2 ,minutes are ok as the page will refresh itself every minute. 
 */
#define HTTP_STATUS_BUSY_TIMEOUT 2

/** Timeout of the player depending on whether the player is playing, or idle. In minutes, values from 1 to 255 are supported. The idle timeout is used when no playback, the play timeout is used while playing. 
 */
#define DEFAULT_IDLE_TIMEOUT 5
#define DEFAULT_PLAY_TIMEOUT 30

/** Default volumes the player will start with. 
 */
#define DEFAULT_VOLUME 5
#define DEFAULT_ALARM_VOLUME 5

/** Walkie-talkie settings. 
 
    MAX_WALKIE_TALKIE_MESSAGES: number of last messages the player remembers and plays back.
    MIN_WALKIE_TALKIE_RECORDING: minimal recording length that would be transmitted (in ms)
 */

#define MAX_WALKIE_TALKIE_MESSAGES 8
#define MIN_WALKIE_TALKIE_RECORDING 1000 * 1

/** UI Timeouts 
 
    All in ticks, the durations of UI actions:

    POWER_ON_PRESS_TICKS: How long a button has to be pressed to wake up from sleep. 
    BUTTON_LONG_PRESS_TICKS: How long a button has to be pressed to become a long press.
    BUTTON_LONG_PRESS_THRESHOLD: How long a button has to be pressed to display the long press progress bar. 
 */
#define POWER_ON_PRESS_TICKS 128
#define BUTTON_LONG_PRESS_TICKS 96
#define BUTTON_LONG_PRESS_THRESHOLD 84

/** Colors
 
    DEFAULT_COLOR: Default color to be used for the UI in the strip. 
    MODE_*: Colors for respective modes.
    BINARY_CLOCK_*: Colors for binary clock parts. 
 */
#define DEFAULT_COLOR Color::White()
#define MODE_COLOR_MP3 Color::Blue()
#define MODE_COLOR_RADIO Color::Green()
#define MODE_COLOR_DISCO Color::Purple()
#define MODE_COLOR_WALKIE_TALKIE Color::Cyan()
#define MODE_COLOR_LIGHTS Color::Yellow()
#define BINARY_CLOCK_HOURS Color::Blue()
#define BINARY_CLOCK_MINUTES Color::Green()
#define BINARY_CLOCK_BATTERY Color::Red()
#define BINARY_CLOCK_CHARGING Color::Cyan()

/** DO NOT CHANGE THE FOLLOWING SETTINGS UNLESS YOU REALLY KNOW WHAT YOU ARE DOING.
 */

/** I2C address of the AVR for communication.
 */
#define AVR_I2C_ADDRESS 67

/** Timeouts
    
    IRQ_RESPONSE_TIMEOUT: If AVR does not hear from ESP after this many ticks (1/64th second) it will restart ESP. 
    ESP_BUSY_TIMEOUT: When it busy mode for more than given number of ticks, the AVR will reset ESP. 
    SYNC_CONNECTION_TIMEOUT: How long (in seconds) ESP will wait for network connection in the sync mode. 
    SPECIAL_LIGHTS_TIMEOUT: in ticks, how long a special lights command result will be dispalyed on the strip. 
 */
#define IRQ_RESPONSE_TIMEOUT 64
#define ESP_BUSY_TIMEOUT 64 * 60 * 1
#define SYNC_CONNECTION_TIMEOUT (120 * 1000)
#define SPECIAL_LIGHTS_TIMEOUT 128

/** Radio frequency min and max values (in Mhz x 10). This must be within the chip's usable range and currently stands at 76.0 - 108.0 Mhz
 */
#define RADIO_FREQUENCY_MIN 760
#define RADIO_FREQUENCY_MAX 1080
/** Max volume of the system + 1. Comes from the radio, as on ESP we can do pretty much any granularity. 
 */
#define MAX_VOLUME 16

/** Voltage thresholds. 

    USB_VOLTAGE_THRESHOLD: Voltage above this level means the player is being powered from the USB charger.
    BATTERY_LOW_VCC: When the battery level drops below this threshold, the low battery flag is raised.
    BATTERY_CRITICAL_VCC: When battery reaches this voltage, the critical alert (three red flashes) will be displayed and the player will go to sleep immediately. 
    BATTERY_DEAD_VCC: When battery reaches this level, no warnings are displayed even and the sleep is immediately resumed. 
 */
#define USB_VOLTAGE_THRESHOLD 440
#define BATTERY_LOW_VCC 350
#define BATTERY_CRITICAL_VCC 340
#define BATTERY_DEAD_VCC 320

/** The battery undervoltage (critical or dead) must be detected consecutively for the specified number of measurements for the action to be taken to prevent the player reacting to quick power fluctuations. 
 
    Upon wakeup, the AVR waits the given ammount of time for the measurements to make sure ESP is not powered up in vain. 
 */
#define UNDERVOLTAGE_TIMEOUT 20

/** Recording settings. 
 
    MAX_RECORDING_LENGTH: Max length of a recording after which it will be stopped and either sent (walkie-talkie), or played (disco).
    RECORDING_GAIN: Amplification of recorded values. Setting this too low will mean inaudible recording, setting it too high means clipping. Depends to the mic board used. 
 */
#define MAX_RECORDING_LENGTH 1000 * 30
#define RECORDING_GAIN 5
