# MP3 Player

A simple and pretty low quality mp3 player, fm radio and a walkie-talkie with some extra features. With stereo headphones (where supported) and a single even lower quality speaker in the box. MP3 files are stored on an SD card, powered by a single Li-Ion 18650 cell, with USB-C port for charging (power only). 

The player can be controlled via two push-button knobs and more advanced settings can be specified via either the player's webpage, or the telegram bot used for the walkie-talkie mode (see below). 

### MP3 Player

Up to 8 playlists can be created, each playlist can handle up to 1024 files. The player imposes no order on the files. 

### Radio

Up to 8 stations can be preset and manual tuning over the applicable range is supported. 

### Walkie-Talkie

Using a dedicated telegram bot, the player can receive WAV files and send recorded WAV files to the same telegram chat, thus effectively working as a walkie-talkie. While audio sent from the player can be played by the telegram clients on android, recording voice in telegram will not be playable on the player as the OGG/Opus codec requires too much memory to fit within ESP8266 limits. Mono uncompressed WAV files must be sent instead. 

New messages are checked every minute when the player is connected, or in between tracks when playing mp3 files (and connected). 

### Night Lights

Flashes the LED strip in various patterns. Can also play specific mp3 file. Alternatively the night-lights 

### Alarm Clock

The alarm clock can only be set from within the web interface, or telegram. At the given time, the player will wake up from sleep and start playing given file (mp3). The alarm is turned off by any button action. 

### Birthay Greeter

Upon first power-on at given date the player will play specified greeting (mp3). Can be set via the web interface or telegram and is supposed to be an easter egg. 



## Setup & Build

`platform.io` is used by both targets. In visual studio, install the platform.io IDE plugin and then simply open the folder.

To be able to program & use the serial monitor, run the following and then restart:

    sudo adduser YOUR_USER dialout

To be able to upload to ATTiny via UPDI, run the following:

    sudo pip install https://github.com/mraardvark/pyupdi/archive/master.zip

Tested on ubuntu 20.04. 

## SD Card

### Player settings

These exist in the `player` folder and consist of the following files:

`player/networks.txt` 

Contains wifi networks and passwords the player will try to connect to when wifi is on. Each network occupies two lines, first line is network name, second line is network password. Leave the line blank for public networks:

    NetworkName1
    Passphrase1
    NetworkName2
    Passphrase2

When WiFi is activated, the networs are tried in order, first successful network on the list will be used. 

`player/ap.txt`

Contains the SSID and password, each on separate line for the access point the player will create if none of the networks in the `wifi.txt` file can be found:

    PlayerAPSSID
    PlayerAPPassword

### Playlists

The SD card contains the following files:


### Radio Settingss

`radio/stations.txt`

Contains up to 8 predefined radio stations, one station per line. The line always starts with the station's frequency `[Mhzx10]`, followed by optional space and name of the station:

    1021 First Station 102.1 Mhz
    950 Second Station

### Walkie-Talkie

The walkie-talkie uses an unique telegram bot that must be assigned to each player. Its configuration files cover the bot identification and a list of telegram chat ids the player will react & send voice to:

`bot/token.txt`

This file contains the telegram bot identification on 2 lines with an optional certificate fingerprint of telegram's servers on line 3. First line contains the bot's id and token separated by `:` as reported by Telegram's bot creation API. The second line contains the chat_id from which commands to the player bot can be sent. The last line may contain the fingerprint for the HTTPS connection, or be left empty for bypassing the check (not secure, use at own risk):

    123456789:AGDHDBDGhsg837498
    674512
    CF 05 98 89 CA FF 8E D8 5E 5C E0 C2 E4 F7 E6

`bot/chats.txt`

Contains up to 8 chat ids that the player can communicate with. Each chat has three elements: the chat id, the color used when playing/recording and the textual description for the ui:

    -635437 0000ff The family
    674512 ff0000 Admin

## Basic Design

The player contains two chips, ATTiny3216 and ESP8266:

- the ATTiny is responsible for power management, user input events, microphone and persisting state
- esp is responsible for playing music, controlling the radio, web access, and the other higher level functions

### ATTiny


### ESP


The attiny is responsible for the controls and device status maintenance (rtc, charging, I/O operations, such as switching audio source, detecting headphones, etc.). The ESP is responsible for actual playback (i.e. the mp3 songs from the SD card a)

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


## TODO

- lights do not always turn themselves off?
- determine better indicator for the audio strength (some annaeling?)

- repeat modes for mp3
- some mp3 files take time to load and make avr reset esp, see why?
- extra settings, such as time limited volume, radio, 

- AGND and GND should be joined at 5V regulator

## AVR

- runs off the battery directly, i.e. can go as low as (2.6V), and as high as input (5V)
- this means that neopixel and rx pin must be level shifted - do they at 3.4V which is the lowest we go? 

## ESP8266 Core



### Exceptions Decoder

- tools line in `~/snap/arduino/current/Arduino/tools`

https://github.com/me-no-dev/EspExceptionDecoder



Use old bootstrap because it is smaller

https://getbootstrap.com/docs/3.4/customize/


MP3 encoder - maybe

https://github.com/zhuker/lamejs

Bootstrap code must be updated to look for fonts in "." and then change the filenames so that they are short enouygh for littlefs (glyphicons.woff2)


