# HW

- enlarge footprints for all ICs (?)
- larger holes for audio jack
- update the audio src, tpa power, and attiny serial port accordingly
- see PCB based connectors for pwr, neopixels and speaker

# ATTiny

- what to do with connected to wifi notification? I do not want it on all the time, but there should be a way how to determine if connected or not

- where to store if walkie-talkie is set (maybe message sth like PeriodicWTSync(on/off))
- control lights don't seem to flash well
- audiolights are a bit weird...
- move code from ISRs to main loop where possible (tick)
- determine decent speed settings for the night light effects

# ESP

- only one channel for the Walkie-Talkie bot 
- approaching memory limit for ESP...
- actually implement downloading telegram file
- wifi connecting gauge is wrong direction
- fix the 1 minute tick properly
- check time and update it
- longer delay before power down in night light mode (done, check)
- wifi_off (http) does not inform about success
- check MDNS
- add authentication
- remove uses of String class, although perhaps not practical, see the memory usage

 # Missing Features

 - alarm clock
 - birthday reminder

# Walkie Talkie

Sound recording: Adafruit MEMS do not work... Tried two and they produce pretty much garbage:(, sparkfun mems gives reasonably good results and so does electret microphone with a transimpedance amplifier. The 8x oversampling really helps, there is still noise, but it is already usable. An actual circuit with all the noise reduction capacitors might help too. 

Maybe add a silence reduction as well (i.e values close to center are center). 