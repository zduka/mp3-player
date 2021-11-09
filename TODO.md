# HW

# SW

- should notification light do stuff if we are in non music mode? 

- do not poweroff if busy, generally, inform the ESP that it should power off. It then sends the power off message to which we react
- radio frequency right after start still does not work well, maybe bigger interval

- headphones can be set to output and LOW to disable speaker any time

- add the following to the settings:
    - power off timeout
    - max speaker volume (4bits)
    - max headphones volume (4bits)
- read from SD card (some JSON)

# ATTiny

- temp is wildly off up to being useless

# ESP

- better integrate time update to the workflow, timezone support
- add telegram commands, such as checking which wifi & ip we have, etc.

- check heap and health and reset when necessary

# Walkie Talkie

- actually implement downloading telegram file
- maybe add a silence reduction as well (i.e values close to center are center). 

# Birthday reminder

First turn on when a date is set, a message is played. Maybe support more dates? 

# Alarm Clock

Enters alarm clock mode, plays the alarm. Short press snoozes, Long press turns off. Can select radio/mp3 from the SD card as the wakeup call. 

# Sync Mode

Run silently, check notifications & update time. When to run? Every N hours, if active, delay until poweroff.