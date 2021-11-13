# HW

# SW

- pressing control when idle? should start play?

- should notification light do stuff if we are in non music mode? 

- radio frequency right after start still does not work well, maybe bigger interval

- headphones can be set to output and LOW to disable speaker any time

# ATTiny

- temp is wildly off up to being useless

# ESP

- simpler playlists management (just enable/disable/etc)
- better integrate time update to the workflow, timezone support

- check heap and health and reset when necessary

- OTA Updates should be checked on proper network and explicitly enabled in the remote

# Walkie Talkie

- maybe add a silence reduction as well (i.e values close to center are center). 
- on initialize load indices so that we know how much messages we have
- play already played messages & advance the id after played

# Birthday reminder

First turn on when a date is set, a message is played. Maybe support more dates? 

# Alarm Clock

Enters alarm clock mode, plays the alarm. Short press snoozes, Long press turns off. Can select radio/mp3 from the SD card as the wakeup call. 

# Sync Mode

Run silently, check notifications & update time. When to run? Every N hours, if active, delay until poweroff.

# Battery Times

9:05 - 11:50 (4.2 - 4.08) ~3h
8:30 - 10:45 (4.08 - 4.02) ~2h 5h
5h?          (4.02 - 3.86) ~5h 10h
8:15 - 11:10 (3.86 - 3.7) ~3h  13h
8:35