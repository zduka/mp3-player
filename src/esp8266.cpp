#if (defined ARCH_ESP8266)
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <LittleFS.h>
#include <SPI.h>
#include <SD.h>
#include <DNSServer.h>
#include <Hash.h>

#include <radio.h>
#include <RDA5807M.h>
#include <AudioFileSourceSD.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>

#include "state.h"
#include "messages.h"
#include "esp8266/wav_writer.h"

/* ESP8266 PINOUT

         -- RST        up   1 -- TXD0
         -- ADC        up   3 -- I2S_DATA (RXD0)
         -- CH_PC           5 -- SCL
      CS -- 16 up           4 -- SDA
    SCLK -- 14         up   0 -- AVR_IRQ
    MISO -- 12         up   2 -- I2S_WS
    MOSI -- 13       down  15 -- I2S_SCK
         -- VCC           GND --
 
 */

#define I2C_SCL 5
#define I2C_SDA 4

#define CS 16
#define AVR_IRQ 0


class Player {
public:
    /** Initializes the player. 

        Goes to deep sleep immediately if SCL and SDL are both pulled low at startup, which attiny uses when it detects there is not enough battery to run properly. 

        1 = input voltage ok
        2 = esp started (after update state)
        3 = SD card found and ok
        4 = settings ok, radio stations ok, playlists ok
        5 = wifi & servers ok
     */
    static void Initialize() {
        // before we do anything, check if we should go to deep sleep immediately conserving power
        pinMode(I2C_SCL, INPUT);
        pinMode(I2C_SDA, INPUT);
        if (!digitalRead(I2C_SCL) && !digitalRead(I2C_SDA))
            ESP.deepSleep(0);
        // continue with normal initialization
        Serial.begin(74880);
        LOG("Initializing ESP8266...");
        LOG("  chip id:      " + ESP.getChipId());
        LOG("  cpu_freq:     " + ESP.getCpuFreqMHz());
        LOG("  core version: " + ESP.getCoreVersion());
        LOG("  SDK version:  " + ESP.getSdkVersion());
        LOG("  mac address:  " + WiFi.macAddress());
        LOG("  wifi mode:    " + WiFi.getMode());
        // set the IRQ pin as input so that we can tell when AVR has an interrupt
        pinMode(AVR_IRQ, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(AVR_IRQ), AVRIrq, FALLING);
        // start the I2C comms
        Wire.begin(I2C_SDA, I2C_SCL);
        Wire.setClock(400000);
        // initialize the on-chip filesystem
        InitializeLittleFS();
        // tell AVR that we are awake by requesting initial state
        UpdateState();
        msg::Send(msg::LightsBar{2, 5, Color::White(),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});



        // initialize the SD card
        InitializeSDCard();
        msg::Send(msg::LightsBar{3, 5, Color::White(),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});

        InitializeSettings();
        InitializeRadioStations();
        InitializeMP3Playlists();
        msg::Send(msg::SetAccentColor{Settings_.accentColor});
        msg::Send(msg::SetMode{State_});
        msg::Send(msg::LightsBar{4, 5, Settings_.accentColor, DEFAULT_SPECIAL_LIGHTS_TIMEOUT});


        //SetMode(Mode::Radio);
        //SetRadioStation(0);
        //VolumeChange(3);

        //Message::Send(Message::SetVolume{0, 16});
        //UpdateState();
        //InitializeAP();
        InitializeWiFi();
        InitializeServer();
        PreviousSecondMillis_ = millis();
        msg::Send(msg::LightsBar{5, 5, Settings_.accentColor,DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
        LOG("Initialization done.");

        //WiFiConnect();
        
        //SetMode(Mode::MP3);
        //SetPlaylist(1);
        //SetTrack(0);

        SetMode(Mode::Radio);
        SetRadioStation(0);

        //SetMode(Mode::NightLight);
        LOG("End of setup");
        delay(100);
        if (!AOut_.begin("/test.wav")) {
            LOG("Cannot open file");
        } else {
            msg::Send(msg::StartRecording{});
            Recording_ = true;
        }
    }

    // TODO update this for proper recording
    static inline uint16_t RecordedLength_ = 0;
    static inline WavWriter AOut_;
    static inline bool Recording_ = false;

    static void Loop() {
        // update the running usage statistics
        uint32_t t = millis();
        ++RunningLoopCount_;
        uint16_t delay = t - LastMillis_;
        if (RunningMaxLoopTime_ < delay)
            RunningMaxLoopTime_ = delay;
        LastMillis_ = t;
        // see if we have a second tick, in which case update the time and utilization counters
        while (t - PreviousSecondMillis_ >= 1000) {
            PreviousSecondMillis_ += 1000;
            SecondTick();
        }

        // see if we need to update the state
        if (Status_.irq) 
            UpdateState();
        // do what we have to - handle DNS, server, mp3 player, etc. 
        DNSServer_.processNextRequest();
        Server_.handleClient();

        // based on the mode, do the bookkeeping
        if (MP3_.isRunning()) {
            if (!MP3_.loop()) {
                MP3_.stop();
                LOG("MP3 playback done");
                PlayNextTrack();
            }
        }
    }

    /** Prepares the ESP to power off and informs AVR to cut power & go to sleep. 
     */
    static void PowerOff() {
        LOG(PSTR("Powering off"));
        msg::Send(msg::PowerOff{});
        // do nothing as AVR is supposed to power off the chip immediately
        while (true) { }
    }

private:

    static void Error(uint8_t code = 0) {
        LOG("ERROR: " + code);
        // TODO tell avr
    }

    static void InitializeLittleFS() {
        LittleFS.begin();
        FSInfo fs_info;
        LittleFS.info(fs_info); 
        LOG("LittleFS Stats:");
        LOG("  Total bytes:     " +fs_info.totalBytes);
        LOG("  Used bytes:      " +fs_info.usedBytes);
        LOG("  Block size:      " +fs_info.blockSize);
        LOG("  Page size:       " +fs_info.pageSize);
        LOG("  Max open files:  " +fs_info.maxOpenFiles);
        LOG("  Max path length: " +fs_info.maxPathLength);
    }

    static void InitializeSDCard() {
        if (SD.begin(CS, SPI_HALF_SPEED)) {
            LOG("SD Card:");
            switch (SD.type()) {
                case 0:
                    LOG("  type: SD1");
                    break;
                case 1:
                    LOG("  type: SD2");
                    break;
                case 2:
                    LOG("  type: SDHC");
                    break;
                default:
                    LOG("  type: unknown");
            }
            LOG("  volume type: FAT" + SD.fatType());
            LOG("  volume size: " + (static_cast<uint32_t>(SD.totalClusters()) * SD.clusterSize() / 1000000) + " [MB]");
        } else {
            LOG("Error reading from SD card");
            Error();
        }
    }
    
    /** Loads the application settings from the SD card. 
     
        The settings are stored in the `settings.txt` file. If the file does not exist, uses the default settings. 
     */
    static void InitializeSettings() {
        // actually set the settings to their defaults first in case of corrupted settings file
        Settings_.accentColor = DEFAULT_ACCENT_COLOR;
        Settings_.maxBrightness = DEFAULT_BRIGHTNESS;
        Settings_.powerOffTimeout = DEFAULT_POWEROFF_TIMEOUT;
        Settings_.wifiTimeout = DEFAULT_WIFI_TIMEOUT;
        Settings_.allowRadioManualTuning = DEFAULT_ALLOW_RADIO_MANUAL_TUNING;
        Settings_.maxSpeakerVolume = DEFAULT_MAX_SPEAKER_VOLUME;
        Settings_.maxHeadphonesVolume = DEFAULT_MAX_HEADPHONES_VOLUME;
        State_.resetVolume(DEFAULT_VOLUME, 16); // volume is from 0 to 15 by HW requirements 
        // TODO actually read this from the SD card & stuff, only upon first round






        Status_.powerOffCountdown = Settings_.powerOffTimeout;
        Status_.wifiCountdown = Settings_.wifiTimeout;
        // check if current volume is too high and if so, adjust accordingly
        uint8_t maxVolume = State_.headphones() ? Settings_.maxHeadphonesVolume : Settings_.maxSpeakerVolume;
        State_.resetVolume(State_.volume() > maxVolume ? maxVolume : State_.volume(), 16);
    }

    /** Called every time a second passes.
     */
    static void SecondTick() {
        LoopCount_ = RunningLoopCount_;
        MaxLoopTime_ = RunningMaxLoopTime_;
        RunningLoopCount_ = 0;
        RunningMaxLoopTime_ = 0;
        //LOG("loops: " + LoopCount_ + ", max diff: " + MaxLoopTime_);
        // disconnect wifi if the wifi inactivity countdown has reached 0
        if (State_.wifiStatus() != WiFiStatus::Off && Status_.wifiCountdown != 0 && --Status_.wifiCountdown == 0)
            WiFiDisconnect();
        if (Status_.idle && State_.wifiStatus() == WiFiStatus::Off && --Status_.powerOffCountdown == 0)
            PowerOff();
    }

    /** Handler for the state update requests.

        These are never done by the interrupt itself, but a flag is raised so that the main loop can handle the actual I2C request when ready. 
     */
    static IRAM_ATTR void AVRIrq() {
        Status_.irq = true;
    }

    /** Gets the state from ATTiny. 
     */
    // TODO update this for proper recording 
    static void UpdateState() {
        Status_.irq = false;
        size_t n = Wire.requestFrom(AVR_I2C_ADDRESS, Recording_ ? 32 : sizeof(State));
        if (Recording_) {
            if (n != 32)
                LOG("Corrupted recording length: " + n);
            else 
                RecordedLength_ += 32;
            while (n-- > 0) 
                AOut_.add(Wire.read());
            if (RecordedLength_ >= 8000) {
                Recording_ = false;
                msg::Send(msg::StopRecording{});
                AOut_.end();
                LOG("Recording done");
            }
        } else if (n == sizeof(State)) {
            State old = State_;
            Wire.readBytes(pointer_cast<uint8_t*>(& State_), n);
            LOG("I2C State: " + State_.voltage() + " [Vx100], " + State_.temp() + " [Cx10], CTRL: " + State_.control() + "/" + State_.maxControl() + ", VOL: " + State_.volume() + "/" + State_.maxVolume());
            // TODO charging 
            // TODO headphones
            // TODO alarm

            if (State_.control() != old.control())
                ControlChange();
            if (State_.controlDown() != old.controlDown())
                State_.controlDown() ? ControlDown() : ControlUp();
            if (State_.controlPress())
                ControlPress();
            if (State_.controlLongPress())
                ControlLongPress();
            if (State_.volume() != old.volume())
                VolumeChange();
            if (State_.volumeDown() != old.volumeDown())
                State_.volumeDown() ? VolumeDown() : VolumeUp();
            if (State_.volumePress())
                VolumePress();
            if (State_.volumeLongPress())
                VolumeLongPress();
            State_.clearButtonEvents();
            // TODO process the events now 
        } else {
            LOG("I2C State corruption: " + n);
        }
    }


    static inline State State_;

    static inline volatile struct {
        bool idle : 1;
        bool irq : 1;
        unsigned powerOffCountdown : 12;
        unsigned wifiCountdown : 12;
    } Status_;

    static inline struct {
        Color accentColor;
        uint8_t maxBrightness : 8;
        unsigned powerOffTimeout : 12;
        unsigned wifiTimeout : 12;
        unsigned nightLightsTimeout : 12;
        bool allowRadioManualTuning : 1;
        unsigned maxSpeakerVolume : 4;
        unsigned maxHeadphonesVolume : 4;
    } Settings_;

    /** Calculates usage statistics for the ESP chip. 
     
        Namely two metrics - the average and maximum distance between two calls to the loop function in milliseconds. 
     */
    static inline uint16_t LoopCount_ = 0;
    static inline uint16_t MaxLoopTime_ = 0;

    static inline uint32_t LastMillis_ = 0;
    static inline uint16_t RunningLoopCount_ = 0;
    static inline uint16_t RunningMaxLoopTime_ = 0;
    static inline uint32_t PreviousSecondMillis_ = 0;

    /** Number of times the loop function gets called in the past second. 

        This number can 
     * 
     * 
     * 55k when completely idle
     * 16k when playing mp3 with hiccups
     * 38k when playing low quality mono mp3 
     * 20k when playing stereo mp3 
     */

/** \name Controls
 
    This section contains handlers for the input events such as rotary encoder changes, presses, headphones, etc. 
 */
//@{
private:

    static void ControlChange() {
        LOG("Control: " + State_.control());
        switch (State_.mode()) {
            case Mode::MP3: {
                if (State_.mp3PlaylistSelection()) {
                    SetPlaylist(State_.control());
                    msg::Send(msg::LightsPoint{State_.control(), State_.maxControl() - 1, MP3_PLAYLIST_COLOR.withBrightness(Settings_.maxBrightness),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
                } else {
                    SetTrack(State_.control());
                    msg::Send(msg::LightsPoint{State_.control(), State_.maxControl() - 1, MP3_TRACK_COLOR.withBrightness(Settings_.maxBrightness),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
                }
                break;
            }
            case Mode::Radio: {
                if (State_.radioManualTuning()) {
                    SetRadioFrequency(State_.control() + RADIO_FREQUENCY_OFFSET);
                    msg::Send(msg::LightsPoint{State_.control(), State_.maxControl() - 1, RADIO_FREQUENCY_COLOR.withBrightness(Settings_.maxBrightness),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
                } else {
                    SetRadioStation(State_.control());
                    msg::Send(msg::LightsPoint{State_.control(), State_.maxControl() - 1, RADIO_STATION_COLOR.withBrightness(Settings_.maxBrightness),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
                }
                break;
            }
            case Mode::NightLight: {
                if (State_.nightLightHueSelection()) {
                    SetNightLightHue(State_.control());
                    if (State_.control() == State::NIGHTLIGHT_RAINBOW_HUE) 
                        msg::Send(msg::LightsColors{
                            Color::HSV(0 << 13, 255, Settings_.maxBrightness),
                            Color::HSV(1 << 13, 255, Settings_.maxBrightness),
                            Color::HSV(2 << 13, 255, Settings_.maxBrightness),
                            Color::HSV(3 << 13, 255, Settings_.maxBrightness),
                            Color::HSV(4 << 13, 255, Settings_.maxBrightness),
                            Color::HSV(5 << 13, 255, Settings_.maxBrightness),
                            Color::HSV(6 << 13, 255, Settings_.maxBrightness),
                            Color::HSV(7 << 13, 255, Settings_.maxBrightness),
                            DEFAULT_SPECIAL_LIGHTS_TIMEOUT                            
                        });
                    else 
                        msg::Send(msg::LightsBar{8, 8, State_.nightLightColor(Settings_.accentColor, Settings_.maxBrightness),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
                } else {
                    SetNightLightEffect(static_cast<NightLightEffect>(State_.control()));
                    msg::Send(msg::LightsPoint{State_.control(), State_.maxControl() - 1, NIGHTLIGHT_EFFECT_COLOR.withBrightness(Settings_.maxBrightness), DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
                }
                break;
            }
        }
    }

    static void ControlDown() {
        LOG("Control down");
    }

    static void ControlUp() {
        LOG("Control up");
    }

    static void ControlPress() {
        LOG("Control press");
        switch (State_.mode()) {
            case Mode::MP3: {
                State_.setMp3PlaylistSelection(!State_.mp3PlaylistSelection());
                SetMode(State_.mode());
                msg::Send(msg::SetMP3Settings{State_});
                break;
            }
            case Mode::Radio:
                State_.setRadioManualTuning(!State_.radioManualTuning());
                SetMode(State_.mode());
                msg::Send(msg::SetMode{State_});
                msg::Send(msg::SetRadioSettings{State_});
                break;
            case Mode::NightLight:
                State_.setNightLightHueSelection(!State_.nightLightHueSelection());
                SetMode(State_.mode());
                msg::Send(msg::SetMode{State_});
                msg::Send(msg::SetNightLightSettings{State_});
                break;
        }
    }

    static void ControlLongPress() {
        LOG("Control long press");
        // TODO switch mode
    }

    /** Volume knob always adjusts the volume. 
     
        No need to update any state here as the avr's state has been updated by the user input causing the change and esp state has been updated already - we are simply reacting to the event. 
     */
    static void VolumeChange() {
        uint8_t maxVolume = State_.headphones() ? Settings_.maxHeadphonesVolume : Settings_.maxSpeakerVolume;
        if (State_.volume() > maxVolume) {
            State_.setVolume(maxVolume);
            msg::Send(msg::SetMode{State_});
        }
        LOG("Volume: " + State_.volume());
        switch (State_.mode()) {
            case Mode::MP3:
            case Mode::WalkieTalkie:
            case Mode::NightLight:
                I2S_.SetGain(State_.volume() * ESP_VOLUME_STEP);
                break;
            case Mode::Radio:
                Radio_.setVolume(State_.volume());
                break;
        }
        msg::Send(msg::LightsBar{State_.volume(), State_.maxVolume() - 1, VOLUME_COLOR.withBrightness(Settings_.maxBrightness),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
    }

    static void VolumeDown() {
        LOG("Volume down");

    }

    static void VolumeUp() {
        LOG("Volume down");

    }

    /** Play/Pause toggle.
     */
    static void VolumePress() {
        LOG("Volume press");
        if (Status_.idle) 
            Play();
        else
            Pause();
    }

    /** Enables or disables the audio lights. 
     */
    static void VolumeLongPress() {
        LOG("Volume long press");
        // TODO switch audio lights on or off
    }




//@}

    /** Changes the player's mode. 
     */
    static void SetMode(Mode mode) {
        // if the new mode is different than the old mode, we must first close the old mode
        if (State_.mode() != mode) {
            switch (State_.mode()) {
                case Mode::MP3:
                case Mode::Radio:
                    Pause();
                    break;
                case Mode::WalkieTalkie:
                    break;
                case Mode::NightLight:
                    break;
            }

        }
        // set the new mode
        State_.setMode(mode);
        switch (mode) {
            case Mode::MP3: {
                LOG("Mode: MP3");
                if (State_.mp3PlaylistSelection()) {
                    State_.resetControl(State_.mp3PlaylistId(), NumPlaylists());
                } else {
                    State_.resetControl(State_.mp3TrackId(), NumTracks(State_.mp3PlaylistId()));
                }
                Play();
                break;
            }
            case Mode::Radio: {
                LOG("Mode: radio");
                if (State_.radioManualTuning()) {
                    State_.resetControl(State_.radioFrequency() - RADIO_FREQUENCY_OFFSET, RADIO_FREQUENCY_MAX);
                } else {
                    State_.resetControl(State_.radioStation(), NumRadioStations());
                }
                Play();
                break;
            }
            case Mode::WalkieTalkie: {
                break;

            }
            case Mode::NightLight: {
                LOG("Mode: NightLight");
                Status_.powerOffCountdown = Settings_.nightLightsTimeout;
                if (State_.nightLightHueSelection()) 
                    State_.resetControl(0, 32);
                else
                    State_.resetControl(0, 5);
                // TODO play a lullaby
                break;
            }
        }


        // finally, inform the AVR of the mode change and other updates
        msg::Send(msg::SetMode{State_});
    }

    /** Resumes playback.
     */ 
    static void Play() {
        switch (State_.mode()) {
            case Mode::MP3:
                break;
            case Mode::Radio:
                Radio_.init();
                Radio_.setMono(true);
                Radio_.setVolume(State_.volume());
                Radio_.setBandFrequency(RADIO_BAND_FM, State_.radioFrequency() * 10);
                break;
            case Mode::NightLight:
                break;
            // don't do anything for walkie talkie ?
        }
        Status_.idle = false;
    }

    /** Pauses or stops playback. 
     */
    static void Pause() {
        switch (State_.mode()) {
            case Mode::MP3:
                if (MP3_.isRunning()) {
                    MP3_.stop();
                    I2S_.stop();
                    MP3File_.close();
                }
                break;
            case Mode::Radio:
                Radio_.term();
                break;
            case Mode::NightLight:
                break;
            // don't do anything for walkie talkie ?
        }
        Status_.idle = true;
        Status_.powerOffCountdown = Settings_.powerOffTimeout;
    }

    /** Looks at the SD card and finds valid playlists. 
     
        Up to 8 playlists are supported, which can be found in directories labelled 1..8. If the directory contains file `playlist.txt`, then it is a valid playlist. The file contains whether the playlist is enabled (1) or hidden (0) followed by the optional name of the playlist. 
     */
    static void InitializeMP3Playlists() {
        LOG("Initializing MP3 Playlists...");
        int pi = 0;
        for (uint8_t i = 1; i <= 8; ++i) {
            File f = SD.open(STR(i + "/playlist.txt"), FILE_READ);
            if (f) {
                int enabled = f.parseInt();
                f.read(); // skip the space (assuming it is space...)
                String name = f.readStringUntil('\n');
                if (enabled) {
                    uint16_t numTracks = GetPlaylistTracks(i);
                    MP3Playlists_[pi] = PlaylistInfo{i, numTracks};
                    ++pi;
                    LOG("  " + i + ": " + name + " (" + numTracks + " tracks)");
                } else {
                    LOG("  " + i + ": " + name + " (disabled)");
                }
            }
        }
        for (; pi < 8; ++pi)
            MP3Playlists_[pi].invalidate();
    }

    /** Returns the number of tracks available for given playlist. 
     
        Assumes the playlist is valid. 
     */
    static uint16_t GetPlaylistTracks(uint8_t playlist) {
        uint16_t result = 0;
        File d = SD.open(STR(playlist));
        while (true) {
            File f = d.openNextFile();
            if (!f)
                break;
            // this should not happen, but let's be sure
            if (f.isDirectory())
                continue;
            if (String{f.name()}.endsWith(".mp3")) 
                ++result;
        }
        if (result > 1023) {
            LOG("Error: Playlist " + playlist + " has " + result + " tracks where only 1023 is allowed");
            result = 1023;
        }
        d.close();
        return result;
    }

    /** Sets the playlist with given index and starts playint its first track. 

        The index is the index to the runtime playlist table, not the id of the playlist on the SD card.  
     */
    static void SetPlaylist(uint8_t index) {
        LOG("Playlist " + index + " (folder " + MP3Playlists_[index].id + ")");
        CurrentPlaylist_.close();
        CurrentPlaylist_ = SD.open(STR(MP3Playlists_[index].id));
        State_.setMp3PlaylistId(index);
        State_.setMp3TrackId(0);
        SetTrack(0);
    }

    /** Starts playing the given track id within the current playlist. 
     */
    static void SetTrack(uint16_t index) {
        // pause current playback if any
        if (MP3_.isRunning()) {
            MP3_.stop();
            I2S_.stop();
            MP3File_.close();
        }
        // determune whether we have to start from the beginning, or can go forward from current track
        uint16_t nextTrack = State_.mp3TrackId() + 1;
        if (index < nextTrack) {
            CurrentPlaylist_.rewindDirectory();
            nextTrack = 0;
        }
        while (true) {
            File f = CurrentPlaylist_.openNextFile();
            if (!f)
                break;
            LOG("Checking file " + f.name() + " index: " + index + " nextTrack: " + nextTrack);
            if (String(f.name()).endsWith(".mp3")) {
                if (index == nextTrack) {
                    String filename{STR(MP3Playlists_[State_.mp3PlaylistId()].id + "/" + f.name())};
                    f.close();
                    MP3File_.open(filename.c_str());
                    I2S_.SetGain(State_.volume() * ESP_VOLUME_STEP);
                    MP3_.begin(& MP3File_, & I2S_);
                    LOG("Track " + index + ", file: " + filename);
                    State_.setMp3TrackId(index);
                    msg::Send(msg::SetMP3Settings{State_});
                    return;
                } else {
                    ++nextTrack;
                }
            } 
            f.close();
        }
        LOG("No file found");
        // TODO error
    }

    static void PlayNextTrack() {
        if (State_.mp3TrackId() + 1 < NumTracks(State_.mp3PlaylistId())) {
            SetTrack(State_.mp3TrackId() + 1);
            SetMode(State_.mode());
            msg::Send(msg::SetMP3Settings{State_});
        } else {
            Pause();
        }
    }

    /** Returns the number of tracks available in the given playlist. 
     */
    static uint16_t NumTracks(uint8_t playlistId) {
        if (playlistId >= 8)
            return 0;
        return MP3Playlists_[playlistId].numTracks;
    }

    /** Returns the available number of mp3 playlists. 
     */
    static uint8_t NumPlaylists() {
        uint8_t i = 0;
        for (; i < 8; ++i)
            if (! MP3Playlists_[i].isValid())
                break;
        return i;
    }

    struct PlaylistInfo {
        unsigned id : 4;
        unsigned numTracks : 10;

        PlaylistInfo():
            id{0},
            numTracks{0} {
        }

        PlaylistInfo(uint8_t id, uint16_t numTracks):
            id{id},
            numTracks{numTracks} {
        }

        bool isValid() const {
            return id != 0;
        }

        void invalidate() {
            id = 0;
        }
    }; // 

    static inline AudioGeneratorMP3 MP3_;
    static inline AudioOutputI2S I2S_;
    static inline AudioFileSourceSD MP3File_;
    static inline PlaylistInfo MP3Playlists_[8];

    static inline File CurrentPlaylist_;

    /** Initializes the predefined radio stations from the SD card. 
     
        The stations are defined in the `stations.txt` file, one station per line, first the frequency and then optional name separated by a new line. 
     */
    static void InitializeRadioStations() {
        LOG("Initializing radio stations...");
        for (int i = 0; i < 7; ++i)
            RadioStations_[i] = RADIO_STATION_NONE;
        File f = SD.open("stations.txt", FILE_READ);
        if (f) {
            int i = 0;
            while (i < 8) {
                uint16_t freq = static_cast<uint16_t>(f.parseInt());
                if (freq == 0)
                    break;
                f.read(); // skip the space (assuming it is space...)
                String name = f.readStringUntil('\n');
                RadioStations_[i] = freq;
                ++i;
                LOG("  " + name + ": " + freq);
            }
            f.close();
        } else {
            LOG("  stations.txt file not found");
        }
    }

    static void SetRadioFrequency(uint16_t mhzx10) {
        LOG("Radio frequency: " + mhzx10);
        if (State_.mode() == Mode::Radio) {
            Radio_.setBandFrequency(RADIO_BAND_FM, mhzx10 * 10);            
            State_.setRadioFrequency(mhzx10);
        }
        msg::Send(msg::SetRadioSettings{State_});
    }

    static void SetRadioStation(uint8_t index) {
        LOG("Radio station: " + index);
        if (State_.mode() == Mode::Radio) {
            Radio_.setBandFrequency(RADIO_BAND_FM, RadioStations_[index] * 10);
            State_.setRadioFrequency(RadioStations_[index]);
            State_.setRadioStation(index);            
        }
        msg::Send(msg::SetRadioSettings{State_});
    }

    /** Returns the number of valid stations. 
     */
    static uint8_t NumRadioStations() {
        uint8_t result = 0;
        while (result < 8 && RadioStations_[result] != RADIO_STATION_NONE)
            ++result;
        return result;
    }

    static inline RDA5807M Radio_;

    static constexpr uint16_t RADIO_STATION_NONE = 0xffff;
    static inline uint16_t RadioStations_[8];

/** \name Night Lights
 */

    static void SetNightLightEffect(NightLightEffect effect) {
        LOG("Night light effect: " + static_cast<int>(effect));
        State_.setNightLightEffect(effect);
        msg::Send(msg::SetNightLightSettings{State_});
    }

    static void SetNightLightHue(uint8_t hue) {
        LOG("Night light hue: " + hue);
        State_.setNightLightHue(hue);
        msg::Send(msg::SetNightLightSettings{State_});
    }

/** \name Webserver
 */
//@{
private:

    static void InitializeWiFi() {
        WiFiConnectedHandler_ = WiFi.onStationModeConnected(OnWiFiConnected);
        WiFiIPAssignedHandler_ = WiFi.onStationModeGotIP(OnWiFiIPAssigned);
        WiFiDisconnectedHandler_ = WiFi.onStationModeDisconnected(OnWiFiDisconnected);
    }

    static void WiFiConnect() {
        LOG("WiFi: Scanning networks...");
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        WiFi.scanNetworksAsync([](int n) {
            LOG("WiFi: Networks found: " + n);
            File f = SD.open("networks.txt", FILE_READ);
            if (f) {
                while (f.available()) {
                    String ssid = f.readStringUntil('\n');
                    String pass = f.readStringUntil('\n');
                    if (ssid.isEmpty())
                        break;
                    for (int i = 0; i < n; ++i) {
                        if (WiFi.SSID(i) == ssid) {
                            LOG("WiFi: connecting to " + ssid + ", rssi: " + WiFi.RSSI(i) + ", channel " + WiFi.channel(i));
                            State_.setWiFiStatus(WiFiStatus::Connecting);
                            msg::Send(msg::SetWiFiStatus{State_});
                            WiFi.begin(ssid, pass);
                            WiFi.scanDelete();
                            // so that we do not start the access point just yet
                            return;
                        }
                    }
                }
                f.close();
            } else {
                LOG("WiFi: No networks.txt found");
            }
            WiFi.scanDelete();
            WiFiStartAP();
        });
    }

    static void WiFiStartAP() {
        File f = SD.open("ap.txt", FILE_READ);
        String ssid;
        String pass;
        if (f) {
            ssid = f.readStringUntil('\n');
            pass = f.readStringUntil('\n');
            f.close();
        } else {
            LOG("WiFi: No ap.txt found");
        }
        if (ssid.isEmpty()) {
            ssid = DEFAULT_AP_SSID;
            if (pass.isEmpty())
                pass = DEFAULT_AP_PASSWORD;
        }
        LOG("Initializing soft AP, ssid " + ssid + ", password " + pass);
        LOG("    own ip: 10.0.0.1");
        LOG("    subnet: 255.255.255.0");
        IPAddress ip{10,0,0,1};
        IPAddress subnet{255, 255, 255, 0};
        WiFi.softAPConfig(ip, ip, subnet);
        if (!WiFi.softAP(ssid.c_str(), pass.c_str())) 
            return Error();            
        State_.setWiFiStatus(WiFiStatus::SoftAP);
        msg::Send(msg::SetWiFiStatus{State_});
    }

    static void WiFiDisconnect() {
        LOG("WiFi: Disconnecting");
        WiFi.mode(WIFI_OFF);
        WiFi.forceSleepBegin();
        yield();
        State_.setWiFiStatus(WiFiStatus::Off);
        msg::Send(msg::SetWiFiStatus{State_});
    }

    static void OnWiFiConnected(WiFiEventStationModeConnected const & e) {
        LOG("WiFi: connected to " + e.ssid + ", channel " + e.channel);
    }

    static void OnWiFiIPAssigned(WiFiEventStationModeGotIP const & e) {
        LOG("WiFi: IP assigned: " + e.ip.toString() + ", gateway " + e.gw.toString());
        State_.setWiFiStatus(WiFiStatus::Connected);
        msg::Send(msg::SetWiFiStatus{State_});
    }

    /** TODO what to do when the wifi disconnects, but we did not initiate it? 
     */
    static void OnWiFiDisconnected(WiFiEventStationModeDisconnected const & e) {
        LOG("WiFi: disconnected, reason: " + e.reason);
    }

    static void InitializeServer() {
        Server_.onNotFound(Http404);
        Server_.serveStatic("/", LittleFS, "/index.html");
        Server_.serveStatic("/favicon.ico", LittleFS, "/favicon.ico");
        Server_.serveStatic("/bootstrap.min.css", LittleFS, "/bootstrap.min.css");
        Server_.serveStatic("/jquery-1.12.4.min.js", LittleFS, "/jquery-1.12.4.min.js");
        Server_.serveStatic("/bootstrap.min.js", LittleFS, "/bootstrap.min.js");
        Server_.serveStatic("/lame.min.js", LittleFS, "/lame.min.js");
        Server_.on("/status", HttpStatus);
        Server_.on("/cmd", HttpCommand);
        Server_.on("/sd", HttpSDCardDownload);
        Server_.on("/sdls", HttpSDCardLs);
        IPAddress ip{10,0,0,1};
        DNSServer_.start(
            53,
            "*",
            ip
        );
        Server_.begin();
        MDNS.begin("mp3-player");
    }

    static void Authenticate() {

    }

    static void Http404() {
        Server_.send(404, "text/json","{ \"response\": 404, \"uri\": \"" + Server_.uri() + "\" }");
    }

    /** Returns the status of the player. 
     */
    static void HttpStatus() {
        // reset the wifi timeout
        Status_.wifiCountdown = Settings_.wifiTimeout;
        Server_.send(200, "text/json", STR(PSTR("{") +
            PSTR("\"millis\":") + millis() + 
            PSTR(",\"rssi\":") + WiFi.RSSI() +
            PSTR(",\"headphones\":") + State_.headphones() +
            PSTR(",\"charging\":") + State_.charging() +
            PSTR(",\"voltage\":") + State_.voltage() +
            PSTR(",\"temp\":") + State_.temp() + 
            PSTR(",\"control\":") + State_.control() +
            PSTR(",\"maxControl\":") + State_.maxControl() +
            PSTR(",\"volume\":") + State_.volume() +
            PSTR(",\"maxVolume\":") + State_.maxVolume() +
            PSTR(",\"mode\":") + static_cast<uint8_t>(State_.mode()) +
            PSTR(",\"audioLights\":") + State_.audioLights() +
            PSTR(",\"wifiStatus\":") + static_cast<uint8_t>(State_.wifiStatus()) +
            PSTR(",\"mp3PlaylistId\":") + State_.mp3PlaylistId() +
            PSTR(",\"mp3TrackId\":") + State_.mp3TrackId() +
            PSTR(",\"mp3PlaylistSelection\":") + State_.mp3PlaylistSelection() +
            PSTR(",\"radioFrequency\":") + State_.radioFrequency() + 
            PSTR(",\"radioStation\":") + State_.radioStation() +
            PSTR(",\"radioManualTuning\":") + State_.radioManualTuning() +
            PSTR(",\"espLoopCount\":") + LoopCount_ +
            PSTR(",\"espMaxLoopTime\":") + MaxLoopTime_ +
        PSTR("}")));
    }

    static void HttpCommand() {
        String const & cmd = Server_.arg("cmd");
        if (cmd == "wifi_off") {
            Server_.send(200, "text/json","{\"response\":200}");
            WiFiDisconnect();
            return;
        } else {
            Server_.send(404, "text/json","{ \"response\": 404, \"unknownCommand\": \"" + cmd + "\" }");
        }
        LOG("Cmd: " + cmd);
        Server_.send(200, "text/json","{\"response\":200}");
    }

    /** Returns any given file on the SD card. 
     */
    static void HttpSDCardDownload() {
        String const & path = Server_.arg("path");
        File f = SD.open(path.c_str(), FILE_READ);
        if (!f || !f.isFile())
            return Http404();
        String ctype = "text/plain";
        if (path.endsWith("mp3"))
            ctype = "audio/mp3";
        LOG("WebServer: Serving file " + path);
        Server_.streamFile(f, ctype);
        f.close();
    }

    /** Lists a directory on the SD card and returns its contents in a JSON format. 
     */
    static void HttpSDCardLs() {
        String const & path = Server_.arg("path");
        File d = SD.open(path.c_str());
        if (!d || !d.isDirectory())
            return Http404();
        LOG("WebServer: Listing directory " + path);
        String r{"["};
        int n = 0;
        while(true) {
            File f = d.openNextFile();
            if (!f)
                break;
            if (n++ > 0)
                r += ",";
            r = r + "{\"name\":\"" + f.name() + "\", \"size\":";
            if (f.isDirectory())
                r += "\"dir\"}";
            else 
                r = r + f.size() + "}";
        } 
        r += "]";
        Server_.send(200, "text/json", r);
    }

    static inline WiFiEventHandler WiFiConnectedHandler_;
    static inline WiFiEventHandler WiFiIPAssignedHandler_;
    static inline WiFiEventHandler WiFiDisconnectedHandler_;

    static inline ESP8266WebServer Server_{80};
    static inline DNSServer DNSServer_;

    static inline String AuthHash_;
    static inline uint32_t AuthTime_;
    static inline uint32_t AuthIP_;


//@}

}; // Player

/** Disable wifi at startup. 
 
    From: https://github.com/esp8266/Arduino/issues/6642#issuecomment-578462867
    and:  https://github.com/esp8266/Arduino/tree/master/libraries/esp8266/examples/LowPowerDemo

    TL;DR; DO as much as possible as soon as possible and start setup() with a delay as the delay is really the thing that makes esp enter the sleep mode. 
 */

RF_PRE_INIT() {
    system_phy_set_powerup_option(2);  // shut the RFCAL at boot
    wifi_set_opmode_current(NULL_MODE);  // set Wi-Fi working mode to unconfigured, don't save to flash
    wifi_fpm_set_sleep_type(MODEM_SLEEP_T);  // set the sleep type to modem sleep
}

void preinit() {
    wifi_fpm_open();
    wifi_fpm_do_sleep(0xffffffff);
}

void setup() {
    delay(1); // necessary to enter the modem sleep mode
    Player::Initialize();
}

void loop() {
    Player::Loop();
}

#endif // ARCH_ESP8266
