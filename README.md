# MP3 Player

A simple and pretty low quality mp3 and fm radio player with the following main features:


## Controls

The player uses two rotary encoders with integrated buttons as the only physical controllers. The buttons support a short press and a long press, which is indicated on the led strip.

To turn the player on, a long press of either control, or volume is required.

> TODO should there be a lock mode where this does not work and some less accidental power on sequence will be used for travel, etc.?

The player can be in three modes - mp3 player, fm radio, and web interface. To change between the mp3 and fm radio modes, long press the control button.

> TODO How to enter the web interface mode? just long press both, or something more elaborate?

The volume controller always controls the audio volume and its press toggles play/pause(mute).

The control has different meaning based on the current mode. In the radio mode, short press triggers station presets, or manual tuning (if allowed). Turns then either change frequency, or present station. In the mp3 mode, 

- long volume press turns the lights on/off? adds color, and so on?

Control: controls which mp3 track, or radion station will be played, switches between mp3 and radio modes.

Volume: controls the volume and play/pause. 

A lot more can be configured via the web interface.

## Basic Design

The attiny is responsible for the controls and device status maintenance (rtc, charging, I/O operations, such as switching audio source, detecting headphones, etc.). The ESP is responsible for actual playback (i.e. the mp3 songs from )

## TODO

- change voltage input to 4.2 - 3.4, low battery warning from 3.5 or 3.6? need to test this, the regulators won't work below 3.4v
- add ADC1 for voltage & charging & headphones
- ADC can be used to get temperature off the chip, which may be cool

- add idle timer and poweroff, add timeout info to state after powered up
- make esp read state from avr after powerup
- determine better indicator for the audio strength (some annaeling?)

- repeat modes for mp3
- some mp3 files take time to load and make avr reset esp, see why?
- extra settings, such as time limited volume, radio, 

- rewrite schematics for AC coupled headphones, make sure the noise is neopixel or breadboard relatred

## AVR

- runs off the battery directly, i.e. can go as low as (2.6V), and as high as input (5V)
- this means that neopixel and rx pin must be level shifted

## ESP8266 Core



### Exceptions Decoder

- tools line in `~/snap/arduino/current/Arduino/tools`

https://github.com/me-no-dev/EspExceptionDecoder

### LittleFS Uploader

https://github.com/earlephilhower/arduino-esp8266littlefs-plugin/releases


## AVR ESP8266 Communication

- esp runs

