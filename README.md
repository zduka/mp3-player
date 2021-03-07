# MP3 Player

A simple and pretty low quality mp3 and fm radio player with the following main features:


## Controls

The player uses two rotary encoders with integrated buttons as the only physical controllers.

Control: controls which mp3 track, or radion station will be played, switches between mp3 and radio modes.

Volume: controls the volume and play/pause. 

A lot more can be configured via the web interface.

## Basic Design

The attiny is responsible for the controls and device status maintenance (rtc, charging, I/O operations, such as switching audio source, detecting headphones, etc.). The ESP is responsible for actual playback (i.e. the mp3 songs from )

Mode | attiny | esp8266 |
-----|--------|---------|
mp3  | slave  | master  |
fm   | master | sleep   |
www  | slave  | master  |



To turn on: press ctrl or volume knob and hold for 

## TODO

- the 3v3 rail drops a lot when in boost mode, see why 

On avr:

- add I2C slave
- make attiny sleep and use RTC one second wakeup
- integrate power, strip and buttons into the player as well
- add ADC and start reading voltage
- add reading the audio out and showing this on the neopixel strip


On esp:

- read from avr

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

