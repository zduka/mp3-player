# HW

- enlarge footprints for all ICs and through-hole components
- larger holes for audio jack
- update the audio src, tpa power, and attiny serial port accordingly
- add LEDs to the attiny & serial port (they can either go to the led strip kind of, or be behind the buttons)
- headphones signal is inverted for TPA311 (!!)
- determine if clean voltage is necessary, or if audio can go via 3V3 already - most likely can go with 3v3 as it is, seems pretty clean on breadboard audio
- no need for charging voltage pin at AVR
- when the inductor has capacitor, it *seems* that the antenna can be connected to ground 
- see PCB based connectors for pwr, neopixels and speaker

# ATTiny

- control lights don't seem to flash well
- audiolights are a bit weird...
- re-enable sleep
- move code from ISRs to main loop where possible (tick)
- record sound when wanted, ignore when not
- determine decent speed settings for the night light effects

# ESP

- when recording, check that the button is still held down
- proper recording code, not just the proof-of-concept hack
- longer delay before power down in night light mode (done, check)
- change strings to PSTR
- wifi_off does not inform about success
- check MDNS
- add authentication

 # Missing Features

 - walkie talkie mode
 - alarm clock
 - birthday reminder

# Walkie Talkie

Sound recording: Adafruit MEMS do not work... Tried two and they produce pretty much garbage:(, sparkfun mems gives reasonably good results and so does electret microphone with a transimpedance amplifier. The 8x oversampling really helps, there is still noise, but it is already usable. An actual circuit with all the noise reduction capacitors might help too. 

Maybe add a silence reduction as well (i.e values close to center are center). 