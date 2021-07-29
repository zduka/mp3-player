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

- re-enable sleep
- move code from ISRs to main loop where possible (tick)
- record sound when wanted, ignore when not
- better lights algorithm - currently is always on almost all the time
- determine decent speed settings for the night light effects
- not sure why but poweron does not power ESP sometimes, forcefully turning esp on fixes everything
- make sure that ADC is disconnected while sleeping
- audio source must be switched accordingly

# ESP

- longer delay before power down in night light mode (done, check)
- change strings to PSTR
- wifi_off does not inform about success
- check MDNS
- add authentication

 # Missing Features

 - walkie talkie mode
 - alarm clock
 - birthday reminder
