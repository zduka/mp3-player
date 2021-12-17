# MP3 Player

A simple and pretty low quality mp3 player, fm radio and a walkie-talkie with some extra features. With stereo headphones (where supported) and a single even lower quality speaker in the box. MP3 files are stored on an SD card, powered by a single Li-Ion 18650 cell, with USB-C port for charging (power only). 

The player can be controlled via two push-button knobs and more advanced settings can be specified via either the player's webpage.

For a "user manual" see the documentation, this readme is for technical information only. 

![Finished player](https://github.com/zduka/mp3-player/blob/master/docs/images/finished.jpg)

# Software

The player uses 2 MCUs - ATTiny and ESP8266. ESP acts as the main controller, while ATTiny takes care of power management and peripherals. Communication between the chips is done via I2C bus. ATTiny and the LED strip runs directly off the battery voltage, while a SMPS to 3v3 provides power for the audio output and the ESP8266. 

In sleep mode, only the ATTiny and input buttons circuitry is powered and ATTiny is in sleep mode, waking up each second for timekeeping. 

> When flashing the ESP chip, make sure power won't be terminated by ATTiny in the middle of the transaction by shorting the 3V3 jumper wire on the pcb. To perform a full reflash of both chips, start with ESP8266, when done don't reset it and flash AtTiny immediately. When AtTiny is done, it will reset (power on) the ESP as well. 

# Hardware & Assembly Guide


## Assembly Guide

> This is also heavily dependent on how tight you are with the parts and how great your soldering skills are. I was tight with parts and my soldering skills are very limited:)

### Enclosure

Glue together the upper part and the bottom corners. Make sure to insert the screw nuts in the appropriate places and glue them. Glue the bottom, left and right plates. Sand and paint the finished middle frame. 

Glue the bottom part of the front plate. Then glue in the translucent plastic strip with epoxy and the top front plate.

### USB-C Li-Ion charger

Desolder both LEDs and then put a 10k resistor (1206 is fine) instead of the charging LED (red, closer to the USB connector). Connect the charging pin to the connector closer to the connector. Desolder R3 and replace it with 2k7 resistor (through hole) directly from GND to terminal 2 of TP4056 to lower the charging current. Desolder the LEDs and replace the one closer to the USB-C connector with a 10k resistor (this will be a pull-up for the charging sense). 

Attach wires (+5V USB, +, -, charging and long connectors to the battery). When fitted in the enclosure, solder the ends to the battery holder. The battery holder is reversed so that it does not have to be glued and the wires won't obstruct the translucent LED strip below. 

### LED Strip

The LED strip consists of 8 neopixel-style LEDs soldered together. Then glue the strip in place. Make sure the glue does not obstruct the screw mountholes. 

### Main PCB

Take the 3v3 SMPS a saw off the mounthole bit and lower part of connector holes to make it castellated. Then solder the regulator, all the ICs (w/o radio and ESP8266), the MOSFETs (2x), diode and inductors. Solder the 3v3 enable header and debug header. Power on via the power connector, check first that ATTiny has power and that power levels are good and power consuption is small. Then short the 3v3 header to enable 3v3 via SMPS and check the voltage and current draw. 

Solder the capacitor for neopixels strip isolation, attach neopixels, and flash ATTiny with the neopixel test (`-DTEST_NEOPIXEL`). The pixels should flash.

Solder the remaining audio path (resistors and caps) and the radio module. Then solder the 3v3 isolation capacitors. Power on and flash ATTiny with `-DTEST_RADIO`. Attach a speaker or headphones and run. Radio should start playing (you might need to change the station frequency if you want to hear stuff). 

Solder everything else on the main board, but not the control knobs. Flash ATTiny and ESP8266 with the normal code, add empty SD card and power on. A welcome MP3 should play. 

Put the knobs in (don't solder yet), and put the whole PCB in the enclosure, attach with screws. Use hard paper strips to center the knobs in the front panel holes properly and when secured, solder the knobs. Check that everything works mechanically.

Solder the microphone board. First the IC, then all SMT parts and finally the microphone. Solder long headers, put these through the holes in pain pcb  board and when the distance is proper, solder them to the main PCB Board as well. 

Solder the antenna wire. 

![Internals](https://github.com/zduka/mp3-player/blob/master/docs/images/internals.jpg)

### Final Assembly

Attach the speaker and slide the PCB with knobs on in the enclosure, attching the neopixels while doing so. Fix with all screws. Secure the power connector. Cover the blue ESP8266 diode. Attach the backplate.

## Main PCB Description




## ESP8266 Core

### Exceptions Decoder

- tools line in `~/snap/arduino/current/Arduino/tools`

https://github.com/me-no-dev/EspExceptionDecoder



Use old bootstrap because it is smaller

https://getbootstrap.com/docs/3.4/customize/


Bootstrap code must be updated to look for fonts in "." and then change the filenames so that they are short enouygh for littlefs (glyphicons.woff2)


