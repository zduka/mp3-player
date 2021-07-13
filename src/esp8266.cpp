#if (defined ARCH_ESP8266)
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <SPI.h>
#include <SD.h>
#include <DNSServer.h>

#include <radio.h>
#include <RDA5807M.h>
#include <AudioFileSourceSD.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2SNoDAC.h>

#include "state.h"
#include "comms.h"

#define LOG(...) Log_(String("") + __VA_ARGS__)
#define STR(...) (String("") + __VA_ARGS__)

inline void Log_(String const & str) {
    Serial.print(String(millis() / 1000) + ": ");
    Serial.println(str);
}

/* ESP8266 PINOUT

         -- RST        up   1 -- TXD0
         -- ADC        up   3 -- I2S_DATA (RXD0)
         -- CH_PC           5 -- SCL
      CS -- 16 up           4 -- SDA
    SCLK -- 14         up   0 -- AVR_IRQ
    MISO -- 12         up   2 -- I2S_WS
    MOSI -- 13       down  15 -- I2S_SCK
         -- VCC           GND --
 
   TODO: Add AUDIO_SRC
   TODO: Add TPA311 power

 */

#define I2C_SCL 5
#define I2C_SDA 4

#define CS 16
#define AVR_IRQ 0


class Player {
public:
    /** Initializes the player. 

     */
    static void Initialize() {
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



        // initialize the SD card
        InitializeSDCard();

        InitializeRadioStations();

        InitializeMP3Playlists();


        //SetMode(Mode::Radio);
        //SetRadioStation(0);
        //VolumeChange(3);

        //Message::Send(Message::SetVolume{0, 16});
        //UpdateState();
        //InitializeAP();
        InitializeWiFi();
        InitializeServer();
        PreviousSecondMillis_ = millis();
        LOG("Initialization done.");


        WiFiConnect();
        //SetMode(Mode::MP3);
        //VolumeChange(4);
        //SetPlaylist(0);
        //SetTrack(0);

    }

    static void Loop() {
        // see if we have a second tick
        if (millis() - PreviousSecondMillis_ >= 1000) {
            PreviousSecondMillis_ += 1000;
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
                // TODO set next track if we are in mp3 mode
            }
        }
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
        LOG("  Total bytes:      " +fs_info.totalBytes);
        LOG("  Used bytes:       " +fs_info.usedBytes);
        LOG("  Block size:       " +fs_info.blockSize);
        LOG("  Page size:        " +fs_info.pageSize);
        LOG("  Max open files:   " +fs_info.maxOpenFiles);
        LOG("  Max path lenghth: " +fs_info.maxPathLength);
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
    
    /** Handler for the state update requests.

        These are never done by the interrupt itself, but a flag is raised so that the main loop can handle the actual I2C request when ready. 
     */
    static ICACHE_RAM_ATTR void AVRIrq() {
        Status_.irq = true;
    }

    /** Gets the state from ATTiny. 
     */
    static void UpdateState() {
        Status_.irq = false;
        size_t n = Wire.requestFrom(AVR_I2C_ADDRESS, sizeof(State));
        if (n == sizeof(State)) {
            Wire.readBytes(pointer_cast<uint8_t*>(& State_), n);
            LOG("I2C State: " + State_.voltage() + " [Vx100], " + State_.temp() + " [Cx10], CTRL: " + State_.control() + "/" + State_.maxControl() + ", VOL: " + State_.volume() + "/" + State_.maxVolume());
            LOG("   Events:");
            // TODO process the events now 
        } else {
            LOG("I2C State corruption: " + n);
        }
    }


    static inline State State_;

    static inline volatile struct {
        bool idle : 1;
        bool irq : 1;
    } Status_;

    static inline unsigned long PreviousSecondMillis_;

/** \name Controls
 
    This section contains handlers for the input events such as rotary encoder changes, presses, headphones, etc. 
 */
//@{
private:

    static void ControlChange(uint16_t value) {
        switch (State_.mode()) {
            case Mode::MP3: {

                break;
            }
            case Mode::Radio: {
                if (State_.radioManualTuning()) 
                    SetRadioFrequency(value + RADIO_FREQUENCY_OFFSET);
                else 
                    SetRadioStation(value);
            }
        }
    }

    static void ControlDown() {

    }

    static void ControlUp() {

    }

    static void ControlPress() {
        switch (State_.mode()) {
            case Mode::MP3: {
                break;
            }
            case Mode::Radio:
                State_.setRadioManualTuning(!State_.radioManualTuning());
                if (State_.radioManualTuning()) 
                    State_.setControl(State_.radioFrequency() - RADIO_FREQUENCY_OFFSET, RADIO_FREQUENCY_MAX);
                else
                    State_.setControl(State_.radioStation(), NumRadioStations());
                Message::Send(Message::SetMode{State_});
                Message::Send(Message::SetRadioSettings{State_});
        }

    }

    static void ControlLongPress() {

    }

    /** Volume knob always adjusts the volume. 
     
        No need to update any state here as the avr's state has been updated by the user input causing the change and esp state has been updated already - we are simply reacting to the event. 
     */
    static void VolumeChange(uint8_t value) {
        LOG("Volume: " + value);
        switch (State_.mode()) {
            case Mode::MP3:
            case Mode::WalkieTalkie:
            case Mode::NightLight:
                I2S_.SetGain(value * ESP_VOLUME_STEP);
                break;
            case Mode::Radio:
                Radio_.setVolume(value);
                break;
        }
    }

    static void VolumeDown() {

    }

    static void VolumeUp() {

    }

    /** Play/Pause toggle.
     */
    static void VolumePress() {
        if (Status_.idle) 
            Play();
        else
            Pause();
    }

    /** Enables or disables the audio lights. 
     */
    static void VolumeLongPress() {

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
                break;
            }
            case Mode::Radio: {
                LOG("Mode: radio");
                Play();
                break;
            }
            case Mode::WalkieTalkie: {

            }
            case Mode::NightLight: {

            }
        }


        // finally, inform the AVR of the mode change and other updates
        Message::Send(Message::SetMode{State_});
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
        LOG("Opening playlist " + index + " id: " + MP3Playlists_[index].id);
        CurrentPlaylist_.close();
        CurrentPlaylist_ = SD.open(STR(MP3Playlists_[index].id));
        State_.setMP3PlaylistId(index);
        SetTrack(0);
    }

    /** Starts playing the given track id within the current playlist. 
     */
    static void SetTrack(uint16_t index) {
        uint16_t lastTrack = State_.mp3TrackId();
        if (index <= lastTrack) {
            lastTrack = 0;
            CurrentPlaylist_.rewindDirectory();
        }
        if (MP3_.isRunning()) {
            MP3_.stop();
            I2S_.stop();
            MP3File_.close();
        }
        while (true) {
            File f = CurrentPlaylist_.openNextFile();
            if (String(f.name()).endsWith(".mp3")) {
                if (index == lastTrack) {
                    String filename{STR(MP3Playlists_[State_.mp3PlaylistId()].id + "/" + f.name())};
                    f.close();
                    MP3File_.open(filename.c_str());
                    //I2S_.SetGain(State_.volume() * ESP_VOLUME_STEP);
                    I2S_.SetGain(1);
                    MP3_.begin(& MP3File_, & I2S_);
                    LOG("Opening track " + index + ", file: " + filename);
                    return;
                }
                ++lastTrack;
            }
            f.close();
        }
        // TODO error
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
    static inline AudioOutputI2SNoDAC I2S_;
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
        if (State_.mode() == Mode::Radio) {
            Radio_.setBandFrequency(RADIO_BAND_FM, mhzx10 * 10);            
            State_.setRadioFrequency(mhzx10);
            Message::Send(Message::SetRadioSettings{State_});
        }
    }

    static void SetRadioStation(uint8_t index) {
        if (State_.mode() == Mode::Radio) {
            Radio_.setBandFrequency(RADIO_BAND_FM, RadioStations_[index] * 10);
            State_.setRadioFrequency(RadioStations_[index]);
            State_.setRadioStation(index);            
            Message::Send(Message::SetRadioSettings{State_});
        }
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


/** \name Webserver
 */
//@{
private:

    static void InitializeWiFi() {
        WiFiConnectedHandler_ = WiFi.onStationModeConnected(OnWiFiConnected);
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
            Error();            
    }

    static void WiFiDisconnect() {
        LOG("WiFi: Disconnecting");
        WiFi.mode(WIFI_OFF);
        WiFi.forceSleepBegin();
        yield();
    }

    static void OnWiFiConnected(WiFiEventStationModeConnected const & e) {
        LOG("WiFi: connected to " + e.ssid + ", channel " + e.channel);
        LOG("WiFi: IP " + WiFi.localIP().toString());
    }

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
    }

    static void Http404() {
        Server_.send(404, "text/json","{ \"response\": 404, \"uri\": \"" + Server_.uri() + "\" }");
    }

    /** Returns the status of the player. 
     */
    static void HttpStatus() {
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
            PSTR(",\"mp3PlaylistId\":") + State_.mp3PlaylistId() +
            PSTR(",\"mp3TrackId\":") + State_.mp3TrackId() +
            PSTR(",\"mp3PlaylistSelection\":") + State_.mp3PlaylistSelection() +
            PSTR(",\"radioFrequency\":") + State_.radioFrequency() + 
            PSTR(",\"radioStation\":") + State_.radioStation() +
            PSTR(",\"radioManualTuning\":") + State_.radioManualTuning() +
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
    static inline WiFiEventHandler WiFiDisconnectedHandler_;

    static inline ESP8266WebServer Server_{80};
    static inline DNSServer DNSServer_;



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
    delay(1);
    Player::Initialize();
}

void loop() {
    Player::Loop();
}




#ifdef HAHA

/** Radio station. 

 */
struct RadioStation {
    uint16_t frequency;
    String name;
};

class Player {
public:
    /** Initializes the player. 

        We can't use the constructor as this interferes with the WiFi. Initializes the peripherals (namely I2C to AVR and the IRQ pin and interrupt) and loads the extra configuration from the SD card. 
     */
    void setup() {
        pinMode(AVR_IRQ, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(AVR_IRQ), AvrIRQ, FALLING);
        Wire.begin(I2C_SDA, I2C_SCL);
        // initialize the settings from SD card
        setupSDCard();
        // and now, get the state from AVR and initialize the peripherals as kept in the AVR state
        setupState();
        //delay(200);
    }

    void loop() {
        switch (state_.mode()) {
        case State::Mode::MP3:
            if (mp3_.isRunning()) {
                if (!mp3_.loop()) {
                    mp3_.stop();
                    LOG("song done");
                    playNextTrack();
                }
            }
            break;
        }
        if (stateUpdate_) {
            State old{state_};
            getStateUpdate();
            updateState(old);
        }
    }

    void setMode(State::Mode mode) {
        if (mode != state_.mode()) {
            leaveMode();
            send(Command::SetMode{mode});
        }
        state_.setMode(mode);
        // TODO disable idle as we start playing immediately
        switch (mode) {
            case State::Mode::MP3:
                LOG("mode: mp3");
                setPlaylist(state_.mp3PlaylistId());
                setMp3PlaylistSelection(state_.mp3PlaylistSelection());
                break;
            case State::Mode::Radio:
                LOG("mode: radio");
                radio_.init();
                radio_.setMono(true);
                radio_.setVolume(state_.audioVolume());
                setRadioFrequency(state_.radioFrequency());
                setRadioManualTuning(state_.radioManualTuning());
                break;
            default:
                LOG("unknown mode: " + (int) mode);
        }
    }

    void play() {
        LOG("play");
        if (state_.idle()) {
            state_.setIdle(false);
            send(Command::SetIdle{false});
        }
        switch (state_.mode()) {
            case State::Mode::MP3:
                setTrack(state_.mp3TrackId());
                break;
            case State::Mode::Radio:
                radio_.init();
                radio_.setMono(true);
                radio_.setVolume(state_.audioVolume());
                setRadioFrequency(state_.radioFrequency());
                break;
        }
    }

    void stop() {
        LOG("stop");
        switch (state_.mode()) {
            case State::Mode::MP3:
                // stop current playback
                if (mp3_.isRunning()) {
                    mp3_.stop();
                    i2s_.stop();
                    mp3File_.close();
                }
                break;
            case State::Mode::Radio:
                // instead of mute, terminate the radio to conserve power
                radio_.term();
                break;
        }
        if (! state_.idle()) {
            state_.setIdle(true);
            send(Command::SetIdle{true});
        }
    }

    /** Plays next track. 

        Depending on the settings, this would play next song, or cycle tracks / albums.

        TODO
     */
    void playNextTrack() {
        if (state_.mp3TrackId() + 1 <  numTracks_)
            setTrack(state_.mp3TrackId() + 1);
        else
            stop();
    }

    /** Sets the volume. 

        If the volume is different than current state also changes the state and informs AVR.
     */
    void setAudioVolume(uint16_t volume) {
        volume = (volume > 15) ? 15 : volume;
        LOG("volume: " + volume);
        switch (state_.mode()) {
            case State::Mode::MP3:
                i2s_.SetGain(volume * ESP_VOLUME_STEP);
                break;
            case State::Mode::Radio:
                radio_.setVolume(volume);
                break;
        }
        if (volume != state_.audioVolume()) {
            state_.setAudioVolume(volume);
            send(Command::SetVolume{volume});
        }
    }

    void setAudioLights(bool value) {
        if (state_.audioLights() != value) {
            LOG("audio lights: " + value);
            state_.setAudioLights(value);
            send(Command::SetAudioLights{state_.audioLights()});
        }
    }

    void setRadioFrequency(uint16_t frequency) {
        LOG("radio frequency: " + frequency);
        if (state_.mode() == State::Mode::Radio)
            radio_.setBandFrequency(RADIO_BAND_FM, frequency * 10);
        if (state_.radioFrequency() != frequency) {
            state_.setRadioFrequency(frequency);
            send(Command::SetRadioState{state_});
        }
    }

    void setRadioStation(uint8_t id) {
        id = (id > 7) ? 7 : id;
        LOG("radio station: " + id);
        setRadioFrequency(radioStations_[id].frequency);
        if (state_.radioStation() != id) {
            state_.setRadioStation(id);
            send(Command::SetRadioState{state_});
        }
    }

    void setRadioManualTuning(bool value) {
        if (state_.radioManualTuning() != value) {
            state_.setRadioManualTuning(value);
            send(Command::SetRadioState{state_});
        }
        LOG("radio manual tuning: " + value);
        if (state_.mode() == State::Mode::Radio) {
            if (value) {
                send(Command::SetControl{state_.radioFrequency() - RADIO_FREQUENCY_OFFSET, RADIO_FREQUENCY_MAX});
            } else {
                send(Command::SetControl{state_.radioStation(), 8});
                setRadioStation(state_.radioStation());
            }
        }
    }

    void setMp3PlaylistSelection(bool value) {
        LOG("mp3 playlist selection: " + value);
        if (state_.mp3PlaylistSelection() != value) {
            state_.setMp3PlaylistSelection(value);
            send(Command::SetMP3State{state_});
        }
        if (state_.mode() == State::Mode::MP3) {
            if (value)
                send(Command::SetControl{state_.mp3PlaylistId(), numPlaylists_});
            else
                send(Command::SetControl{state_.mp3TrackId(), numTracks_});
        }
    }
    
private:

    void setupSDCard() {
        // determine song banks
        if (SD.begin(CS, SPI_HALF_SPEED)) {
            LOG("SD card detected, analyzing:");
            numPlaylists_ = 1;
            for (; numPlaylists_ <= 8; ++numPlaylists_) {
                File f = SD.open(String("/") + numPlaylists_);
                if (!f)
                    break;
                f.close();
            }
            --numPlaylists_;
            LOG("Playlists detected: " + numPlaylists_);
        } else {
            LOG("Error reading from SD card. MP3 mode will be disabled.");
        }
        // TODO replace this with actual stations
        radioStations_[0].frequency = 937; // city
        radioStations_[1].frequency = 1050; // vltava
        radioStations_[2].frequency = 1025; // frekvence 1
        radioStations_[3].frequency = 1007; // CRO
        radioStations_[4].frequency = 997; // Bonton 
        radioStations_[5].frequency = 987; // Classic
        radioStations_[6].frequency = 972; // Fajn 
        radioStations_[7].frequency = 966; // Impuls
    }

    /** Sets up the player state based on the latest state stored in avr. 
     */
    void setupState() {
        LOG("Loading state from AVR");
        getStateUpdate();
        setMode(state_.mode());
    }

    void getStateUpdate() {
        stateUpdate_ = false;
        size_t n = Wire.requestFrom(AVR_I2C_ADDRESS,sizeof(State));
        if (n == sizeof(State)) {
            Wire.readBytes(pointer_cast<uint8_t*>(& state_), n);
            LOG("I2C state received");
            LOG("voltage: " + state_.voltage());
            LOG("temp C: " + state_.temperature());
            LOG("temp K64: " + state_.temperatureKelvin());
            LOG("control: " + state_.control());
            LOG("track id: " + state_.mp3TrackId());
            LOG("headphones: " + state_.audioHeadphones());
        } else {
            LOG("I2C status corruption: " + n);
        }
    }
    

    /** Updates the peripherals according to the state changes. 
     */
    void updateState(State & old) {

        // check volume and control first as they can be changed by the rest

        // if volume has changed, update the volume
        if (old.audioVolume() != state_.audioVolume())
            setAudioVolume(state_.audioVolume());
        
        // if control has changed, determine what is the control's function and update accordingly
        if (old.control() != state_.control()) {
            switch (state_.mode()) {
                case State::Mode::MP3:
                    if (state_.mp3PlaylistSelection()) {
                        if (state_.mp3PlaylistId() != state_.control())
                            setPlaylist(state_.control());
                    } else {
                        if (state_.mp3TrackId() != state_.control())
                            setTrack(state_.control());
                    }
                    break;
                case State::Mode::Radio:
                    if (state_.radioManualTuning())
                        setRadioFrequency(state_.control() + RADIO_FREQUENCY_OFFSET);
                    else
                        setRadioStation(state_.control());
                    break;
            }
        }
        
        // react to events, since these will be cleared at the end, there is no need to check them against the old state
        // short press of volume button starts / stops the music 
        if (state_.volumePress())
            state_.idle() ? play() : stop();

        if (state_.volumeLongPress())
            setAudioLights(! state_.audioLights());

        // control press changes playlist in mp3 mode and toggles manual and station tuning in radio
        if (state_.controlPress()) {
            switch (state_.mode()) {
                case State::Mode::MP3:
                    setMp3PlaylistSelection(! state_.mp3PlaylistSelection());
                    break;
                case State::Mode::Radio:
                    setRadioManualTuning(!state_.radioManualTuning());
                    break;
            }
        }

        // control long press changes mode
        if (state_.controlLongPress()) {
            switch (state_.mode()) {
                case State::Mode::MP3:
                    setMode(State::Mode::Radio);
                    break;
                case State::Mode::Radio:
                    setMode(State::Mode::MP3);
                    break;
                
            }
        }

        // clear the processed events and store the state as own
        state_.clearEvents();
    }

    void leaveMode() {
        switch (state_.mode()) {
            case State::Mode::MP3:
                LOG("leaving: mp3");
                if (mp3_.isRunning()) {
                    mp3_.stop();
                    i2s_.stop();
                    mp3File_.close();
                }
                break;
            case State::Mode::Radio:
                LOG("leaving: radio");
                radio_.term();
                break;
        }
    }

    void setPlaylist(unsigned i) {
        if (i >= numPlaylists_)
            i = 0;
        LOG("Opening playlist: " + i);
        state_.setMp3PlaylistId(i);
        currentPlaylist_.close();
        String playlistDir{String{"/"} + (state_.mp3PlaylistId() + 1)};
        currentPlaylist_ = SD.open(playlistDir.c_str());
        numTracks_ = 0;
        while (true) {
            File f = currentPlaylist_.openNextFile();
            // check if there are more files
            if (!f)
                break;
            if (f.isDirectory())
                continue;
            if (String{f.name()}.endsWith(".mp3")) {
                LOG("    " + f.name());
                ++numTracks_;
            }
        }
        LOG("tracks: " + numTracks_);
        currentPlaylist_.rewindDirectory();
        state_.setMp3PlaylistId(i);
        // if current track id is greater than tracks found, reset
        if (state_.mp3TrackId() >= numTracks_)
            state_.setMp3TrackId(0);
        // set the track as well
        setTrack(state_.mp3TrackId());
        // TODO
        //else
        //    send(Command::)
        
    }

    void setTrack(unsigned i) {
        if (numTracks_ == 0) {
            LOG("Cannot play from empty playlist");
            // TODO flash error
            return;
        }
        if (i >= numTracks_)
            i = numTracks_ - 1;
        state_.setMp3TrackId(i);
        // this is super extra not efficient, but whatever, seek before the file to open
        currentPlaylist_.rewindDirectory();
        while (i != 0) {
            File f = currentPlaylist_.openNextFile();
            if (String(f.name()).endsWith(".mp3"))
                --i;
            f.close();
        }
        // stop the current playback and start the new track, if running
        if (mp3_.isRunning()) {
            mp3_.stop();
            i2s_.stop();
            mp3File_.close();
        }
        if (! state_.idle()) {
            // open the file and start playing it
            File f = currentPlaylist_.openNextFile();
            String fileName{String{"/"} + (state_.mp3PlaylistId() + 1) + "/" + f.name()};
            LOG("Playing file: " + fileName);
            mp3File_.open(fileName.c_str());
            f.close();
            i2s_.SetGain(state_.audioVolume() * ESP_VOLUME_STEP);
            mp3_.begin(& mp3File_, & i2s_);
        }
        send(Command::SetMP3State{state_});
    }

    template<typename T>
    void send(T msg) {
        Wire.beginTransmission(AVR_I2C_ADDRESS);
        Wire.write(T::Id);
        Wire.write(pointer_cast<char *>(& msg), sizeof(T));
        Wire.endTransmission();
    }

    /** The AVR stored state. 
     */
    volatile bool stateUpdate_ = false;
    State state_;
    
    /** \name Radio
     */
    //@{
    RDA5807M radio_;
    RadioStation radioStations_[8];
    //@}

    /** \name MP3
     */
    //@{
    AudioGeneratorMP3 mp3_;
    AudioOutputI2SNoDAC i2s_;
    AudioFileSourceSD mp3File_;
    unsigned numPlaylists_;
    File currentPlaylist_;
    unsigned numTracks_; // number of tracks in current playlist
    //@}

    static ICACHE_RAM_ATTR void AvrIRQ();

    //static double Gain_[16];

    
}; // Player



Player player;


/** Handler for the avr irq. 
 */
ICACHE_RAM_ATTR void Player::AvrIRQ() {
    // Serial.println("IRQ");
    player.stateUpdate_ = true;
}


void setup() {
    Core::Setup(/* disableWifi */ true);
    player.setup();
    
    //Core::Connect({SSID1,PASSWORD1, SSID2, PASSWORD2});
    //Server.on("/authenticate", server::authenticate);
    //Server.on("/command", server::command);
    //Core::Server.begin();

    //    pinMode(4, OUTPUT);
    //LOG("Setup done.");


    //Core::DeepSleep();
}

void loop() {
    player.loop();
}




#endif // HAHA


#endif
