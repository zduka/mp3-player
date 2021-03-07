#pragma once

/** The first status register
 */
struct Status {
    unsigned irq : 1;
    unsigned dcdcPower : 1;
};

struct Power {
    unsigned charging : 1;
    /** Shows the current input voltage. 

        Is constructed by reading the input voltage in scale 0..255 corresponding to 0..5v and then discarding the most signifficant byte as under normal conditions, this will always be 1. 

        127 == 5v from external source (and likely charging)
        86  == 4.2v, a fully charged li-ion battery
        60  == 3.7v, nominal value for 
        40  == 3.3v, value at which the low battery warning is given
        25  == 3v, value at which attiny goes immediately to deepsleep again
    */
    unsigned voltage : 7;
};

struct Audio {
    /** Current volume setting. 
     */
    unsigned volume : 4;
    /** Determines the audio source. 

        0 = no audio
        1 = esp8266 mp3 player
        2 = RDA5807 FM radio
    */
    unsigned source : 2;
    /** If 1, headphones are inserted, if 0, speaker will be used instead. Can only be read, and writes will be masked.
     */
    unsigned headphones : 1;

    Audio():
        volume(15),
        source(0),
        headphones(0) {
    }
};

/** Real-time clock and alarms. 
 */
struct Clock {
    unsigned year : 7; // 0..127, corresponding to 2020 to 2147
    unsigned month : 4; // 1..12
    unsigned day : 5; // 1..31
    unsigned h : 6; // 0..59
    unsigned m : 6; // 0..59
    unsigned s : 6; // 0..59
    unsigned alarmDay : 3; // 0..7
    unsigned alarmH : 6; // 0..59
    unsigned alarmM : 6; // 0..59
};

