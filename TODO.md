# HW



- enlarge footprints for all ICs and through-hole components
- larger holes for audio jack
- update the audio src, tpa power, and attiny serial port accordingly
- add LEDs to the attiny & serial port (they can either go to the led strip kind of, or be behind the buttons)
- esp8266 PWM needs jumper to be either RX, or PWM
- headphones signal is inverted for TPA311 (!!)
- determine if the antenna can be connected to ground directly
- determine if clean voltage is necessary, or if audio can go via 3V3 already
- no need for charging voltage pin at AVR

# ATTiny

- not sure why but poweron does not power ESP sometimes, forcefully turning esp on fixes everything

- make speed & night lights mode part of state
- how to make color & speed selectable for night mode? 
- make sure that ADC is disconnected while sleeping

# ESP

- longer delay before power down in night light mode
- change strings to PSTR
- wifi_off does not inform about success
- check MDNS
- add authentication
