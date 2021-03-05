# MP3 Player

To turn on: press ctrl or volume knob and hold for 

## TODO

- determine how control woudl work

On avr:

- make attiny sleep and use RTC one second wakeup
- integrate power, strip and buttons into the player as well
- add ADC and start reading voltage
- add reading the audio out and showing this on the neopixel strip
- add I2C slave


On esp:

- disable wifi by default
- can esp8266 be i2c slave?


### AVR

- runs off the battery directly, i.e. can go as low as (2.6V), and as high as input (5V)
- this means that neopixel and rx pin must be level shifted

## ESP8266 Core



### Exceptions Decoder

- tools line in `~/snap/arduino/current/Arduino/tools`

https://github.com/me-no-dev/EspExceptionDecoder

### LittleFS Uploader

https://github.com/earlephilhower/arduino-esp8266littlefs-plugin/releases



