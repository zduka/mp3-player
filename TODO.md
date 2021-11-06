# HW

- buy the extra things needed

- V_USB should be pulled low otherwise the switch for battery won't work due to the diode leaking... This was done by the v divider in previous version and so was not a problem, but now is. Maybe this won't be an issue if there is pull-down on the charger's IC, check that


# SW

- notifications could perhaps be part of state as opposed to extended state? 

- radio frequency right after start still does not work well, maybe bigger interval

- should idle lights always show status? 

- headphones can be set to output and LOW to disable speaker any time

- add the following to the settings:
    - power off timeout
    - max speaker volume (4bits)
    - max headphones volume (4bits)
- read from SD card (some JSON)

# ATTiny

- temp is wildly off up to being useless

- what to do with connected to wifi notification? I do not want it on all the time, but there should be a way how to determine if connected or not

- where to store if walkie-talkie is set (maybe message sth like PeriodicWTSync(on/off))

# ESP

- control values seem to be off
- audiolights in recording are wrong
- progress bar for sending audio messages, ignore small messages, etc.

- check heap and health and reset when necessary

- only one channel for the Walkie-Talkie bot 
- actually implement downloading telegram file
- check time and update it
- add authentication

 # Missing Features

 - alarm clock
 - birthday reminder

 - a way to record a message to be played directly from the player? Or updateable via telegram?

# Walkie Talkie

Maybe add a silence reduction as well (i.e values close to center are center). 