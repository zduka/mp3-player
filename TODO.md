# HW



- enlarge footprints for all ICs and through-hole components
- larger holes for audio jack
- update the audio src, tpa power, and attiny serial port accordingly
- add LEDs to the attiny & serial port (they can either go to the led strip kind of, or be behind the buttons)
- esp8266 PWM needs jumper to be either RX, or PWM (or maybe not when we use I2S DAC)
- headphones signal is inverted for TPA311 (!!)
- determine if clean voltage is necessary, or if audio can go via 3V3 already
- no need for charging voltage pin at AVR
- no need for inductor at neopixels, or needs to have capacitor too
- when the inductor has capacitor, it *seems* that the antenna can be connected to ground 
- see PCB based connectors for pwr, neopixels and speaker

# ATTiny

- determine decent speed settings for the night light effects
- not sure why but poweron does not power ESP sometimes, forcefully turning esp on fixes everything
- make sure that ADC is disconnected while sleeping

# ESP

- longer delay before power down in night light mode (done, check)
- change strings to PSTR
- wifi_off does not inform about success
- check MDNS
- add authentication
