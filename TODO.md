# HW

- antenna should go on the top only as far away from neopixel rail as possible
- buy the extra things needed
- smaller divider for I2S to audio
- some resistors for the headphones to lower volume

- audio feedback opamp can have bigger gain, most likely. Needs checking...
- where to put the extra LC filter for audio?

- V_USB should be pulled low otherwise the switch for battery won't work due to the diode leaking... This was done by the v divider in previous version and so was not a problem, but now is. Maybe this won't be an issue if there is pull-down on the charger's IC, check that
- check charging detection & charging power dissipation, low battery, AC power - lower charging power to ~0.5A to be on the safe side (2k7), remove LEDs

- radio antenna seems to pickup some LED stuff, should only be on the upper PCB half

- determine useful headphone max volume (R7, R12). Setting this to non-zero may also mean that we can use smaller value for the headphone decoupling caps (C10, C14)

# SW

- should idle lights always show status? 

- headphones can be set to output and LOW to disable speaker any time

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


heap 48k


- check heap and health and reset when necessary

- only one channel for the Walkie-Talkie bot 
- approaching memory limit for ESP...
- actually implement downloading telegram file
- check time and update it
- longer delay before power down in night light mode
- add authentication

 # Missing Features

 - alarm clock
 - birthday reminder

 - a way to record a message to be played directly from the player? Or updateable via telegram?

# Walkie Talkie

Sound recording: Adafruit MEMS do not work... Tried two and they produce pretty much garbage:(, sparkfun mems gives reasonably good results and so does electret microphone with a transimpedance amplifier. The 8x oversampling really helps, there is still noise, but it is already usable. An actual circuit with all the noise reduction capacitors might help too. 

Maybe add a silence reduction as well (i.e values close to center are center). 