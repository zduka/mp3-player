# HW

- verify the I2S output vdiv (R3 + R10, R8 + R14) values so that the volume is similar to the radio
- determine if low pass filter after audio selection is useful, and its values (R4 + C13, R13 + C19)
- determine TPA311 amplification (max speaker & headphone volume when speaker in case - R1, R11)
- determine useful headphone max volume (R7, R12). Setting this to non-zero may also mean that we can use smaller value for the headphone decoupling caps (C10, C14)
- check charging detection & charging power dissipation, low battery, AC power


# SW


- PINS changed for ATTiny to free TXD port
- set radio frequency to first radio station upon complete start

- add the following to the settings:
    - power off timeout
    - max speaker volume (4bits)
    - max headphones volume (4bits)
- read from SD card (some JSON)

# ATTiny


- what to do with connected to wifi notification? I do not want it on all the time, but there should be a way how to determine if connected or not

- where to store if walkie-talkie is set (maybe message sth like PeriodicWTSync(on/off))
- determine decent speed settings for the night light effects

# ESP

- check heap and health and reset when necessary

- only one channel for the Walkie-Talkie bot 
- approaching memory limit for ESP...
- actually implement downloading telegram file
- wifi connecting gauge is wrong direction
- check time and update it
- longer delay before power down in night light mode
- wifi_off (http) does not inform about success
- add authentication

 # Missing Features

 - alarm clock
 - birthday reminder

# Walkie Talkie

Sound recording: Adafruit MEMS do not work... Tried two and they produce pretty much garbage:(, sparkfun mems gives reasonably good results and so does electret microphone with a transimpedance amplifier. The 8x oversampling really helps, there is still noise, but it is already usable. An actual circuit with all the noise reduction capacitors might help too. 

Maybe add a silence reduction as well (i.e values close to center are center). 