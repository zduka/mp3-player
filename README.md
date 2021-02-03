# MP3 Player


- 5V switch needs pull-up to deactivate, active low
- 3V3 switch has pull-up, but needs pull-down, will be active high

### AVR

- runs off the battery directly, i.e. can go as low as (2.6V), and as high as input (5V)
- this means that neopixel and rx pin must be level shifted

### DFPlayer

- power the card through a diode (and dmp1045 perhaps) so that it can take VCC from either dfplayer, or ESP 

## ESP8266 Core



### Exceptions Decoder

- tools line in `~/snap/arduino/current/Arduino/tools`

https://github.com/me-no-dev/EspExceptionDecoder

### LittleFS Uploader

https://github.com/earlephilhower/arduino-esp8266littlefs-plugin/releases



