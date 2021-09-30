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
#include <ArduinoJson.h>

#include <radio.h>
#include <RDA5807M.h>
#include <AudioFileSourceSD.h>
#include <AudioGeneratorMP3.h>
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>

#include "config.h"
#include "state.h"
#include "messages.h"
#include "esp8266/wav_writer.h"
#include "esp8266/telegram_bot.h"

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


#define ERROR LOG

class Player {
public:

    /** Initializes the player
     */
    static void initialize() {
        // initialize serial port for debugging
        Serial.begin(74880);
        // before we do anything, check if we should go to deep sleep immediately conserving power
        pinMode(I2C_SCL, INPUT);
        pinMode(I2C_SDA, INPUT);
        if (!digitalRead(I2C_SCL) && !digitalRead(I2C_SDA)) {
            LOG("Entering deep sleep immediately");
            ESP.deepSleep(0);
        }
        // initialize the IRQ pin and attach interrupt
        pinMode(AVR_IRQ, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(AVR_IRQ), avrIrq, FALLING);
        // enable I2C and request state
        Wire.begin(I2C_SDA, I2C_SCL);
        Wire.setClock(400000);
        updateState();
        //send(msg::LightsBar{2,8,DEFAULT_COLOR.withBrightness(ex_.nightmaxBrightness_)});
        // get the whole extended state, making sure we fit in the buffer
        getExtendedState(ex_);
        ex_.log();
        // Initialize the core subsystems and report back
        initializeESP();
        initializeLittleFS();
        initializeSDCard();
        // 
        initializePlaylists();
        initializeRadioStations();
        //initializeSettings();


        // the extended state could have changed when initializing from the SD card, store it all
        setExtendedState(ex_);

        setMode(Mode::Radio);

        LOG("Free heap: %u", ESP.getFreeHeap());
    }

    static void loop() {
        if (status_.irq) 
            status_.recording ? record() : updateState();
    }

private:

    /** \name Chip & Peripherals Initialization routines
     
        Functions that initialize the ESP chip, image's little fs, attached SD card, etc. 
     */
    //@{
    static void initializeESP() {
        LOG("Initializing ESP8266...");
        LOG("  chip id:      %u", ESP.getChipId());
        LOG("  cpu_freq:     %u", ESP.getCpuFreqMHz());
        LOG("  core version: ", ESP.getCoreVersion().c_str());
        LOG("  SDK version:  %s", ESP.getSdkVersion());
        LOG("  mac address:  %s", WiFi.macAddress().c_str());
        LOG("  wifi mode:    %u", WiFi.getMode());
        LOG("Free heap: %u", ESP.getFreeHeap());
    }

    static void initializeLittleFS() {
        LittleFS.begin();
        FSInfo fs_info;
        LittleFS.info(fs_info); 
        LOG("LittleFS Stats:");
        LOG("  Total bytes:     %u", fs_info.totalBytes);
        LOG("  Used bytes:      %u", fs_info.usedBytes);
        LOG("  Block size:      %u", fs_info.blockSize);
        LOG("  Page size:       %u", fs_info.pageSize);
        LOG("  Max open files:  %u", fs_info.maxOpenFiles);
        LOG("  Max path length: %u", fs_info.maxPathLength);
    }

    static bool initializeSDCard() {
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
            LOG("  volume type: FAT%u", SD.fatType());
            LOG("  volume size: %u [MB]",  (static_cast<uint32_t>(SD.totalClusters()) * SD.clusterSize() / 1000000));
            return true;
        } else {
            LOG("Error reading from SD card");
            return false;
        }
    }

    /** Initializes the global settings from the SD card.

         
     */
    static void initializeSettings() {

    }
    //@}
    
    /** \name Communication with ATTiny
     */
    //@{

    /** Handler for the state update requests.

        These are never done by the interrupt itself, but a flag is raised so that the main loop can handle the actual I2C request when ready. 
     */
    static IRAM_ATTR void avrIrq() {
        status_.irq = true;
    }

    template<typename T>
    static void send(T const & msg) {
        Wire.beginTransmission(AVR_I2C_ADDRESS);
        unsigned char const * msgBytes = pointer_cast<unsigned char const *>(& msg);
        Wire.write(msgBytes, sizeof(T));
        uint8_t status = Wire.endTransmission();
        if (status != 0)
            ERROR("I2C command failed: %u", status);
    }

    template<typename T>
    static void setExtendedState(T const & part) {
        Wire.beginTransmission(AVR_I2C_ADDRESS);
        uint8_t offset = static_cast<uint8_t>((uint8_t*)(& part) - (uint8_t *)(& ex_));
        msg::SetExtendedState msg{
            offset,
            sizeof(T)
        };
        Wire.write(pointer_cast<uint8_t const *>(& msg), sizeof(msg::SetExtendedState));
        Wire.write(pointer_cast<unsigned char const *>(& part), sizeof(T));
        uint8_t status = Wire.endTransmission();
        if (status != 0)
            ERROR("I2C set extended state failed: %u, sending %u bytes", status, sizeof(T));
    }

    template<typename T>
    static void getExtendedState(T & part) {
        Wire.beginTransmission(AVR_I2C_ADDRESS);
        Wire.write(static_cast<uint8_t>((uint8_t*)(& part) - (uint8_t *)(& ex_) + sizeof(State)));
        Wire.endTransmission(false); // repeated start
        size_t n = Wire.requestFrom(AVR_I2C_ADDRESS, sizeof(part));
        if (n == sizeof(part))
            Wire.readBytes(pointer_cast<uint8_t *>(& part), n);
        else
            ERROR("Incomplete transaction while reading extended state, %u bytes received", n);
    }

    /** Requests state update from ATTiny. 
     */
    static void updateState() {
        // set IRQ to false before we get the state. It is possibile that the IRQ will be set to true after this but before the state request, thus being cleared on AVR side by the request. This is ok as it would only result in an extra state request afterwards, which would be identical, and therefore no action will be taken
        status_.irq = false;
        size_t n = Wire.requestFrom(AVR_I2C_ADDRESS, sizeof(State));
        if (n == sizeof(State)) {
            State old = state_;
            Wire.readBytes(pointer_cast<uint8_t*>(& state_), sizeof(State));
            // check available events and react accordingly
            if (state_.controlValue() != old.controlValue()) 
                controlTurn();
            if (state_.controlButtonDown() != old.controlButtonDown())
                state_.controlButtonDown() ? controlDown() : controlUp();
            if (state_.controlButtonPress())
                controlPress();
            if (state_.controlButtonLongPress())
                controlLongPress();
            if (state_.volumeValue() != old.volumeValue())
                volumeTurn();
            if (state_.volumeButtonDown() != old.volumeButtonDown())
                state_.volumeButtonDown() ? volumeDown() : volumeUp();
            if (state_.volumeButtonPress())
                volumePress();
            if (state_.volumeButtonLongPress())
                volumeLongPress();
            if (state_.doubleButtonLongPress())
                doubleLongPress();
            // clear the events so that they can be triggered again (ATTiny did so after the transmission as well)
            state_.clearEvents();
        } else {
            ERROR("Incomplete transaction while reading state, %u bytes received", n);
            // clear the incomplete transaction from the wire buffer
            while (n-- > 0) 
                Wire.read();
        }

    }

    //@}

    /** \name Controls
     
        Events corresponding directly to user interface events as reported by the ATTiny. Methods in this section translate the raw ui events to actionable items based on the current mode & other context. 
     */
    //@{

    static void controlDown() {
        LOG("Control down");

    }

    static void controlUp() {
        LOG("Control up");

    }

    /** Cycles through playlists in mp3 mode, cycles through stations in radio mode, cycles through latest received messages in walkie talkie mode (and plays them) and cycles through light effects for night light mode. 
     */
    static void controlPress() {
        LOG("Control press");
        switch (state_.mode()) {
            case Mode::MP3: {
                uint8_t i = ex_.mp3.playlistId + 1;
                if (i >= numPlaylists_)
                    i = 0;
                setPlaylist(i);
                send(msg::LightsPoint{i, numPlaylists_ - 1, MODE_COLOR_MP3});
                break;
            }
            case Mode::Radio: {
                uint8_t i = ex_.radio.stationId + 1;
                if (i >= numRadioStations_)
                    i = 0;
                setRadioStation(i);
                send(msg::LightsPoint{i, numRadioStations_ - 1, MODE_COLOR_RADIO});
                break;
            }
            case Mode::WalkieTalkie: {
                break;
            }
            case Mode::NightLight: {
                uint8_t i = static_cast<uint8_t>(ex_.nightLight.effect) + 1;
                if (i == 7)
                    i = 0;
                setNightLightEffect(static_cast<NightLightEffect>(i));
                send(msg::LightsPoint{i, 6, MODE_COLOR_NIGHT_LIGHT});
                break;
            }
            default:
                break;
        }
    }

    /** Cycles through the modes. 
     */
    static void controlLongPress() {
        LOG("Control long press");
        setMode(ex_.getNextMode(state_.mode()));
    }

    /** Selects track id in mp3 mode, radio frequency in radio, or changes hue in night lights mode. 
     
        Does nothing in walkie-talkie, can select received messages to play? 
     */
    static void controlTurn() {
        LOG("Control: %u", state_.controlValue());
        switch (state_.mode()) {
            case Mode::MP3:
                send(msg::LightsPoint{state_.controlValue(), ex_.mp3.playlistId, MODE_COLOR_MP3});
                setTrack(state_.controlValue());
                break;
            case Mode::Radio:
                send(msg::LightsPoint{state_.controlValue(), RADIO_FREQUENCY_MAX - RADIO_FREQUENCY_MIN, MODE_COLOR_RADIO});
                setRadioFrequency(state_.controlValue() + RADIO_FREQUENCY_MIN);
                break;
            case Mode::WalkieTalkie:
                break;
            case Mode::NightLight: {
                setNightLightHue(state_.controlValue());
                if (ex_.nightLight.hue == NightLightState::HUE_RAINBOW)
                    send(msg::LightsColors{
                        Color::HSV(0 << 13, 255, ex_.settings.maxBrightness),
                        Color::HSV(1 << 13, 255, ex_.settings.maxBrightness),
                        Color::HSV(2 << 13, 255, ex_.settings.maxBrightness),
                        Color::HSV(3 << 13, 255, ex_.settings.maxBrightness),
                        Color::HSV(4 << 13, 255, ex_.settings.maxBrightness),
                        Color::HSV(5 << 13, 255, ex_.settings.maxBrightness),
                        Color::HSV(6 << 13, 255, ex_.settings.maxBrightness),
                        Color::HSV(7 << 13, 255, ex_.settings.maxBrightness)
                    });
                else
                    send(msg::LightsBar{8, 8, Color::HSV(ex_.nightLight.colorHue(), 255, ex_.settings.maxBrightness)});
                break;
            }
            default:
                break;
        }
    }

    /** Starts recording in walkie-talkie mode. 
     */
    static void volumeDown() {
        LOG("Volume down");

    }

    static void volumeUp() {
        LOG("Volume up");
    }

    /** Play/Pause in mp3, radio and night-lights mode. 
     
        Does nothing in walkie-talkie mode as volume key press & hold is used to record messages. 
      */
    static void volumePress() {
        LOG("Volume press");
        switch (state_.mode()) {
            case Mode::MP3:
            case Mode::Radio:
            case Mode::NightLight:
                if (state_.idle())
                    play();
                else
                    pause();
                break;
            case Mode::WalkieTalkie:
            default:
                break;
        }
    }


    static void volumeLongPress() {
        LOG("Volume long press");
    }

    /** Changes volume. 
     */
    static void volumeTurn() {
        LOG("Volume: %u", state_.volumeValue());
        switch (state_.mode()) {
            case Mode::MP3:
                mp3UpdateVolume();
                break;
            case Mode::Radio:
                radioUpdateVolume();
                break;
            case Mode::WalkieTalkie:
            case Mode::NightLight:
            default:
                break;
        }
    }

    /** Turns wifi on/off, turns the player off. 
     */
    static void doubleLongPress() {
        LOG("Double long press");

    }

    //@}

    /** \name Actions
     
        Actual things that can happen. These can be triggered by various sources, such as the controls, web interface, or the telegram bot. 
     */
    //@{

    static void setControlRange(uint16_t value, uint16_t max) {
        LOG("Control range update: %u (max %u)", value, max);
        if (value > max) {
            value = max;
            LOG("  mac value clipped to %u", max);
        }
        state_.setControlValue(value);
        send(msg::SetControlRange{value, max});
    }
    
    /** Enters the provided mode. 
     */
    static void setMode(Mode mode) {
        LOG("Setting mode %u", mode);
        if (state_.mode() == mode)
            return;
        pause(/* changeIdle */ false);
        state_.setMode(mode);
        send(msg::SetMode(mode));
        play(/* changeIdle */ false);
    }

    static void play(bool changeIdle = true) {
        LOG("Play");
        if (changeIdle) {
            state_.setIdle(false);
            send(msg::SetIdle{false});
        }
        switch (state_.mode()) {
            case Mode::MP3:
                mp3Play();
                break;
            case Mode::Radio:
                radioPlay();
                break;
            case Mode::WalkieTalkie:
                break;
            case Mode::NightLight:
                nightLightPlay();
                break;
            default:
                break;
        }

    }

    static void pause(bool changeIdle = true) {
        LOG("Pause");
        if (changeIdle) {
            state_.setIdle(true);
            send(msg::SetIdle{true});
        }
        switch (state_.mode()) {
            case Mode::MP3:
                mp3Pause();
                break;
            case Mode::Radio:
                radioPause();
                break;
            case Mode::WalkieTalkie:
                break;
            case Mode::NightLight:
                nightLightPause();
                break;
            default:
                break;
        }

    }

    static void setVolume() {
    }

    //@}

    /** \name MP3 Mode 
     */
    //@{
    static void initializePlaylists() {
        LOG("Initializing MP3 Playlists...");
        File f = SD.open("player/playlists.json", FILE_READ);
        if (f) {
            StaticJsonDocument<1024> json;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                numPlaylists_ = 0;
                for (JsonVariant playlist : json.as<JsonArray>()) {
                    uint8_t id = playlist["id"];
                    uint16_t numTracks = initializePlaylistTracks(id);
                    if (numTracks > 0) {
                        playlists_[numPlaylists_++] = PlaylistInfo{id, numTracks};
                        LOG("  %u: %s (%u tracks)",id, playlist["name"].as<char const *>(), numTracks);
                    } else {
                        LOG("  %u: %s (no tracks found, skipping)", id, playlist["name"].as<char const *>());
                    }
                }
            } else {
                ERROR("player/playlists.json is not valid JSON file");
            }
            f.close();
        } else {
            LOG("player/playlists.json not found");
        }
        // disable mp3 mode if no playlists are found
        if (numPlaylists_ == 0)
            ex_.settings.setMp3Enabled(false);
    }

    /** Searches the playlist to determine the number of tracks it contains. 
     
        A track is any file ending in `.mp3`. Up to 1023 tracks per playlist are supported in theory, but much smaller numbers should be used. 
     */
    static uint16_t initializePlaylistTracks(uint8_t playlistId) {
        uint16_t result = 0;
        char playlistFolder[2] = {playlistId + '0', 0};
        File d = SD.open(pointer_cast<char const *>(& playlistFolder));
        while (true) {
            File f = d.openNextFile();
            if (!f)
                break;
            // this should not happen, but let's be sure
            if (f.isDirectory())
                continue;
            if (EndsWith(f.name(), ".mp3"))
                ++result;
        }
        if (result > 1023) {
            LOG("Error: Playlist %u has %u tracks where only 1023 is allowed", playlistId, result);
            result = 1023;
        }
        d.close();
        return result;
    }

    static void mp3Play() {
        setPlaylist(ex_.mp3.playlistId);
    }

    static void mp3Pause() {
        if (mp3_.isRunning()) {
            mp3_.stop();
            i2s_.stop();
            mp3File_.close();
        }
    }

    static void mp3UpdateVolume() {
        if (mp3_.isRunning()) {
            i2s_.SetGain(state_.volumeValue() / 15);
        }
    }

    /** Sets the given playlist and starts playback from its first track. 
     
        Uses the playlist index, which is the order in which valid playlists in `playtlists.json` were found. 

        Assumes the playlist is valid, which is actually checked by the initializePlaylists. 
     */ 
    static void setPlaylist(uint8_t index) {
        LOG("Playlist %u (folder %u)", index, playlists_[index].id);
        currentPlaylist_.close();
        char playlistDir[2] = { playlists_[index].id + '0', 0 };
        currentPlaylist_ = SD.open(playlistDir);
        ex_.mp3.playlistId = index;
        ex_.mp3.trackId = 0;
        setControlRange(0, playlists_[index].numTracks);
        setTrack(0);
    }

    static void setTrack(uint16_t index) {
        // pause current playback if any
        mp3Pause();
        // determune whether we have to start from the beginning, or can go forward from current track
        uint16_t nextTrack = ex_.mp3.trackId + 1;
        if (index < nextTrack) {
            currentPlaylist_.rewindDirectory();
            nextTrack = 0;
        }
        while (true) {
            File f = currentPlaylist_.openNextFile();
            if (!f)
                break;
            LOG("Checking file %s, index %u, nexttrack %u", f.name(), index, nextTrack);
            if (EndsWith(f.name(), ".mp3")) {
                if (index == nextTrack) {
                    char  filename[16]; // this should be plenty for 8.3 filename
                    filename[0] = playlists_[ex_.mp3.playlistId].id + '0';
                    filename[1] = '/';
                    memcpy(filename + 2, f.name(), strlen(f.name()) + 1);
                    f.close();
                    mp3File_.open(filename);
                    i2s_.SetGain(state_.volumeValue() / 15);
                    mp3_.begin(& mp3File_, & i2s_);
                    LOG("Track %u, file: %s", index, filename);
                    ex_.mp3.trackId = index;
                    setExtendedState(ex_.mp3);
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

    static void playNextTrack() {
        if (ex_.mp3.trackId + 1 < playlists_[ex_.mp3.playlistId].numTracks) {
            setTrack(ex_.mp3.trackId + 1);
        } else {
            pause();
        }
    }

    struct PlaylistInfo {
        unsigned id : 4;
        unsigned numTracks : 12;

        PlaylistInfo() = default;

        PlaylistInfo(uint8_t id, uint16_t numTracks):
            id{id},
            numTracks{numTracks} {
        }

    } __attribute__((packed)); // PlaylistInfo

    static_assert(sizeof(PlaylistInfo) == 2);

    static inline AudioGeneratorMP3 mp3_;
    static inline AudioOutputI2S i2s_;
    static inline AudioFileSourceSD mp3File_;
    static inline PlaylistInfo playlists_[8];
    static inline uint8_t numPlaylists_ = 0;

    static inline File currentPlaylist_;

    //@}

    /** \name Radio Mode
     */
    //@{
    /** Initializes the predefined radio stations from the SD card. 

     */
    static void initializeRadioStations() {
        LOG("Initializing radio stations...");
        numRadioStations_ = 0;
        File f = SD.open("radio/stations.json", FILE_READ);
        if (f) {
            StaticJsonDocument<1024> json;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                for (JsonVariant station : json.as<JsonArray>()) {
                    radioStations_[numRadioStations_++] = station["freq"];
                    LOG("  %s: %u", station["name"].as<char const *>(), station["freq"].as<unsigned>());
                }
            } else {
                //LOG("  radio/stations.json file not found");
            }
            f.close();
        } else {
            LOG("  radio/stations.json file not found");
        }
        // if the current frequency is out of bounds, set it to the frequency of the first station, which just means that when invalid state, default to the first station
        if (ex_.radio.frequency < RADIO_FREQUENCY_MIN || ex_.radio.frequency > RADIO_FREQUENCY_MAX)
            ex_.radio.frequency = radioStations_[0];
    }

    static void radioPlay() {
        radio_.init();
        radio_.setMono(ex_.settings.radioForceMono() || ! state_.headphonesConnected());
        radio_.setVolume(state_.volumeValue());
        setRadioFrequency(ex_.radio.frequency);
        setControlRange(ex_.radio.frequency - RADIO_FREQUENCY_MIN, RADIO_FREQUENCY_MAX - RADIO_FREQUENCY_MIN);
    }

    static void radioPause() {
        radio_.term();
    }

    static void radioUpdateVolume() {
        if (state_.mode() == Mode::Radio) {
            radio_.setVolume(state_.volumeValue());
        }
    }

    static void setRadioFrequency(uint16_t mhzx10) {
        LOG("Radio frequency: %u", mhzx10);
        if (state_.mode() == Mode::Radio) {
            ex_.radio.frequency = mhzx10;
            radio_.setBandFrequency(RADIO_BAND_FM, mhzx10 * 10);            
            setExtendedState(ex_.radio);
        }
    }

    static void setRadioStation(uint8_t index) {
        LOG("Radio station: %u", index);
        if (state_.mode() == Mode::Radio) {
            ex_.radio.stationId = index;
            ex_.radio.frequency = radioStations_[index];
            radio_.setBandFrequency(RADIO_BAND_FM, ex_.radio.frequency * 10);
            setExtendedState(ex_.radio);
            setControlRange(ex_.radio.frequency - RADIO_FREQUENCY_MIN, RADIO_FREQUENCY_MAX - RADIO_FREQUENCY_MIN);
        }
    }

    /** NOTE This uses about 500B RAM, none of which we really need as the only relevant radio state is kept in the extended state. This can be saved if we talk to the radio chip directly. 
     */
    static inline RDA5807M radio_;
    static inline uint8_t numRadioStations_ = 0;
    static inline uint16_t radioStations_[8];

    //@}

    /** \name Walkie-Talkie Mode
     */
    //@{
        
    static void stopRecording(bool cancel = false) {
        if (status_.recording) {
            // tell avr to return to normal state & stop own recording
            send(msg::StopRecording{});
            recording_.end();
            if (! cancel) {
                LOG("Recording done");
                // TODO
            } else {
                LOG("Recording cancelled");
                // TODO
            }
        }
    }

    static void record() {
        status_.irq = false;
        size_t n = Wire.requestFrom(AVR_I2C_ADDRESS, 33); // 32 for recorded audio + 1 for state's first byte
        if (n == 33) {
            // add data to the output file
            for (uint8_t i = 0; i < 32; ++i)
                recording_.add(Wire.read());
            // get the first bit of state (we can update our state with it)
            ((uint8_t*)(& state_))[0] = Wire.read();
            // if the volume button has been released, stop recording
            if (! state_.volumeButtonDown())
                stopRecording();
            // otherwise if the control button has been pressed, cancel the recorded sound
            else if (state_.controlButtonDown())
                stopRecording(/* cancel */ true);
        } else {
            ERROR("Incomplete transaction while recording audio, %u bytes received", n);
            // clear the incomplete transaction from the wire buffer
            while (n-- > 0) 
                Wire.read();
        }
    }

    static inline WavWriter recording_;

    //@}

    /** \name Night Lights mode
     */
    //@{

    static void nightLightPlay() {
        setControlRange(ex_.nightLight.hue, NightLightState::HUE_RAINBOW + 1);
        // TODO play lullaby? 
    }

    static void nightLightPause() {
        // TODO stop playing lullaby?
    }

    static void setNightLightEffect(NightLightEffect effect) {
        LOG("Night light effect: %u", static_cast<uint8_t>(effect));
        ex_.nightLight.effect = effect;
        setExtendedState(ex_.nightLight);
    }

    static void setNightLightHue(uint8_t hue) {
        LOG("Night light hue: %u", hue);
        ex_.nightLight.hue = hue;
        setExtendedState(ex_.nightLight);
    }

    //@}

    /** \name Alarm Clock mode
     */
    //@{

    //@}

    /** \name Birthday Greeting mode
     */
    //@{

    //@}

    /** \name Sync mode
     */
    //@{

    //@}




    static inline volatile struct {
        bool irq : 1;
        bool recording : 1;

    } status_;

    static inline State state_;
    static inline ExtendedState ex_;

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
    Player::initialize();
}

void loop() {
    Player::loop();
}







#ifdef HAHA

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
        LOG("Free heap: %u", ESP.getFreeHeap());
        LOG("Initializing ESP8266...");
        LOG("  chip id:      %u", ESP.getChipId());
        LOG("  cpu_freq:     %u", ESP.getCpuFreqMHz());
        LOG("  core version: ", ESP.getCoreVersion().c_str());
        LOG("  SDK version:  %s", ESP.getSdkVersion());
        LOG("  mac address:  %s", WiFi.macAddress().c_str());
        LOG("  wifi mode:    %u", WiFi.getMode());
        // set the IRQ pin as input so that we can tell when AVR has an interrupt
        pinMode(AVR_IRQ, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(AVR_IRQ), AVRIrq, FALLING);
        // start the I2C comms
        Wire.begin(I2C_SDA, I2C_SCL);
        Wire.setClock(400000);
        LOG("Free heap: %u", ESP.getFreeHeap());
        // initialize the on-chip filesystem
        InitializeLittleFS();
        LOG("Free heap: %u", ESP.getFreeHeap());
        // tell AVR that we are awake by requesting initial state
        UpdateState();
        msg::Send(msg::LightsBar{2, 5, Color::White(),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});

        // initialize the SD card
        InitializeSDCard();
        LOG("Free heap: %u", ESP.getFreeHeap());
        msg::Send(msg::LightsBar{3, 5, Color::White(),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});

        InitializeSettings();
        InitializeMP3Playlists();
        InitializeRadioStations();
        InitializeTelegramBot();
        LOG("Free heap: %u", ESP.getFreeHeap());
        msg::Send(msg::SetAccentColor{Settings_.accentColor});
        msg::Send(msg::SetMode{State_});
        msg::Send(msg::LightsBar{4, 5, Settings_.accentColor, DEFAULT_SPECIAL_LIGHTS_TIMEOUT});

        InitializeWiFi();
        InitializeServer();
        PreviousSecondMillis_ = millis();
        msg::Send(msg::LightsBar{5, 5, Settings_.accentColor,DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
        LOG("Initialization done.");
        LOG("Free heap: %u", ESP.getFreeHeap());
        
        SetMode(State_.mode());
        // if wifi was turned on last time, turn it on as well
        if (State_.wifiStatus() != WiFiStatus::Off)
            WiFiConnect();
        LOG("End of setup");
    }

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
        LOG("Powering off");
        msg::Send(msg::PowerOff{});
        // do nothing as AVR is supposed to power off the chip immediately
        while (true) { }
    }

private:

    static void Error(uint8_t code = 0) {
        LOG("ERROR: %u", code);
        // TODO tell avr
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
        static uint8_t seconds = 0;
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
        seconds = (seconds + 1) % 60;
        if (State_.mode() == Mode::WalkieTalkie && seconds == 0)
            WalkieTalkieCheckUpdates();
    }

    /** Handler for the state update requests.

        These are never done by the interrupt itself, but a flag is raised so that the main loop can handle the actual I2C request when ready. 
     */
    static IRAM_ATTR void AVRIrq() {
        Status_.irq = true;
    }

    /** Gets the state from ATTiny. 
     */
    /*
    static void UpdateState() {
        Status_.irq = false;
        size_t n = 0;
        if (Status_.recording) {
            n = Wire.requestFrom(AVR_I2C_ADDRESS, 33);
            if (n == 33) {
                // add data to the output file
                for (uint8_t i = 0; i < 32; ++i)
                    Recording_.add(Wire.read());
                // get the first bit of state and determine if we should stop recording or not
                uint8_t controlState = Wire.read();
                if (! (controlState & State::VOLUME_DOWN_MASK))
                    WalkieTalkieStopRecording();
                else if (controlState & State::CONTROL_DOWN_MASK)
                    WalkieTalkieStopRecording(/* cancel * / true);
                return;
            }
        } else {
            n = Wire.requestFrom(AVR_I2C_ADDRESS, sizeof(State));
            if (n == sizeof(State)) {
                State old = State_;
                Wire.readBytes(pointer_cast<uint8_t*>(& State_), sizeof(State));
                LOG("I2C State: %u [Vx100], %u [Cx10], CTRL: %u/%u, VOL: %u/%u", State_.voltage(), State_.temp(), State_.control(), State_.maxControl(), State_.volume(), State_.maxVolume());
                // TODO charging 
                // TODO headphones
                // TODO alarm ...................

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
                if (State_.doubleLongPress())
                    DoubleLongPress();
                State_.clearButtonEvents();
                return;
            }
        }
        LOG("I2C Err: %u", n);
    } */


    static inline State State_;

    static inline volatile struct {
        bool idle : 1;
        bool irq : 1;
        bool recording : 1;
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
        LOG("Control: %u", State_.control());
        switch (State_.mode()) {
            case Mode::MP3: {
                SetTrack(State_.control());
                msg::Send(msg::LightsPoint{State_.control(), State_.maxControl() - 1, MP3_TRACK_COLOR.withBrightness(Settings_.maxBrightness),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
                break;
            }
            case Mode::Radio: {
                SetRadioFrequency(State_.control() + RADIO_FREQUENCY_OFFSET);
                msg::Send(msg::LightsPoint{State_.control(), State_.maxControl() - 1, RADIO_FREQUENCY_COLOR.withBrightness(Settings_.maxBrightness),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
                break;
            }
            case Mode::NightLight: {
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

    /** Depending on the current mode, short control press does the following:
     
        MP3: Cycles through available playlists
     */
    static void ControlPress() {
        LOG("Control press");
        switch (State_.mode()) {
            case Mode::MP3: {
                uint8_t numPlaylists = NumPlaylists();
                uint8_t playlist = State_.mp3PlaylistId() + 1;
                if (playlist >= numPlaylists)
                    playlist = 0;
                SetPlaylist(playlist);
                msg::Send(msg::LightsPoint{playlist, numPlaylists - 1, MP3_PLAYLIST_COLOR.withBrightness(Settings_.maxBrightness),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
                msg::Send(msg::SetMP3Settings{State_});
                break;
            }
            case Mode::Radio: {
                uint8_t numStations = NumRadioStations();
                uint8_t station = State_.radioStation() + 1;
                if (station >= numStations)
                    station = 0;
                SetRadioStation(station);
                msg::Send(msg::LightsPoint{station, numStations - 1, RADIO_STATION_COLOR.withBrightness(Settings_.maxBrightness),DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
                msg::Send(msg::SetRadioSettings{State_});
                break;
            }
            case Mode::WalkieTalkie: {
                break;
            }
            case Mode::NightLight: {
                uint8_t numEffects = static_cast<uint8_t>(NightLightEffect::Sentinel);
                uint8_t effect = static_cast<uint8_t>(State_.nightLightEffect()) + 1;
                if (effect >= numEffects)
                    effect = 0;
                State_.setNightLightEffect(static_cast<NightLightEffect>(effect));
                msg::Send(msg::LightsPoint{effect, numEffects - 1, NIGHTLIGHT_EFFECT_COLOR.withBrightness(Settings_.maxBrightness), DEFAULT_SPECIAL_LIGHTS_TIMEOUT});
                msg::Send(msg::SetNightLightSettings{State_});
                break;
            }
        }
        // update the mode settings, control & volume scales, etc.
        SetMode(State_.mode());
    }

    /** Long press of the control button cycles through the available modes. 
     */ 
    static void ControlLongPress() {
        LOG("Control long press");
        switch (State_.mode()) {
            case Mode::MP3:
                SetMode(Mode::Radio);
                break;
            case Mode::Radio:
                if (WalkieTalkieModeEnabled())
                    SetMode(Mode::WalkieTalkie);
                else
                    // TODO change this to nightlight in production
                    SetMode(Mode::WalkieTalkie);
                    //SetMode(Mode::NightLight);
                break;
            case Mode::WalkieTalkie:
                SetMode(Mode::NightLight);
                break;
            case Mode::NightLight:
                SetMode(Mode::MP3);
                break;
            // in all other cases, namely alarm clock & birthday greeter, move to the mp3 player
            default:
                SetMode(Mode::MP3);
        }
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
        LOG("Volume: %u", State_.volume());
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
        if (State_.mode() == Mode::WalkieTalkie) {
            if (!State_.controlDown())
                WalkieTalkieStartRecording();
        }
    }

    static void VolumeUp() {
        LOG("Volume up");
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
        State_.setAudioLights(! State_.audioLights());
        msg::Send(msg::SetAudioLights{State_});
    }

    /** Enables or disables the WiFi and with it the walkie-talkie mode. 
     */
    static void DoubleLongPress() {
        LOG("Double long press");
        if (State_.wifiStatus() == WiFiStatus::Off) 
            WiFiConnect();
        else
            WiFiDisconnect();
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
                case Mode::NightLight:
                    Pause();
                    break;
                case Mode::WalkieTalkie:
                    break;
            }

        }
        // set the new mode
        State_.setMode(mode);
        switch (mode) {
            case Mode::MP3: {
                LOG("Mode: MP3");
                State_.resetControl(State_.mp3TrackId(), NumTracks(State_.mp3PlaylistId()));
                Play();
                break;
            }
            case Mode::Radio: {
                LOG("Mode: radio");
                State_.resetControl(State_.radioFrequency() - RADIO_FREQUENCY_OFFSET, RADIO_FREQUENCY_MAX);
                Play();
                break;
            }
            case Mode::WalkieTalkie: {
                LOG("Mode: Walkie-talkie");
                break;
            }
            case Mode::NightLight: {
                LOG("Mode: NightLight");
                Status_.powerOffCountdown = Settings_.nightLightsTimeout;
                State_.resetControl(0, 32);
                // TODO play a lullaby ?
                break;
            }
        }

        // finally, inform the AVR of the mode change and other updates
        msg::Send(msg::SetMode{State_});
    }

    /** Resumes playback.
     */ 
    static void Play() {
        msg::Send(msg::Play{});
        switch (State_.mode()) {
            case Mode::MP3:
                SetPlaylist(State_.mp3PlaylistId());
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
        msg::Send(msg::Pause{});
        Status_.idle = true;
        Status_.powerOffCountdown = Settings_.powerOffTimeout;
    }

    /** Looks at the SD card and finds valid playlists. 
     
        Up to 8 playlists are supported, which can be found in directories labelled 1..8. If the directory contains file `playlist.txt`, then it is a valid playlist. The file contains whether the playlist is enabled (1) or hidden (0) followed by the optional name of the playlist. 
     */
    static void InitializeMP3Playlists() {
        LOG("Initializing MP3 Playlists...");
        for (size_t i = 0; i < 8; ++i)
            MP3Playlists_[i].invalidate();
        File f = SD.open("player/playlists.json", FILE_READ);
        if (f) {
            DynamicJsonDocument json{1024};
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                uint8_t i = 0;
                for (JsonVariant playlist : json.as<JsonArray>()) {
                    uint8_t id = playlist["id"];
                    uint16_t numTracks = GetPlaylistTracks(id);
                    MP3Playlists_[i++] = PlaylistInfo{id, numTracks};
                    LOG("  %u: %s (%u tracks)",id, playlist["name"].as<char const *>(), numTracks);
                }
            } else {

            }
            f.close();
        }
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
            LOG("Error: Playlist %u has %u tracks where only 1023 is allowed", playlist, result);
            result = 1023;
        }
        d.close();
        return result;
    }

    /** Sets the playlist with given index and starts playint its first track. 

        The index is the index to the runtime playlist table, not the id of the playlist on the SD card.  
     */
    static void SetPlaylist(uint8_t index) {
        LOG("Playlist %u (folder %u)", index, MP3Playlists_[index].id);
        CurrentPlaylist_.close();
        CurrentPlaylist_ = SD.open(STR(MP3Playlists_[index].id));
        State_.setMp3PlaylistId(index);
        State_.setMp3TrackId(0);
        SetTrack(0);
        State_.resetControl(0, NumTracks(State_.mp3PlaylistId()));
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
            LOG("Checking file %s, index %u, nexttrack %u", f.name(), index, nextTrack);
            if (String(f.name()).endsWith(".mp3")) {
                if (index == nextTrack) {
                    String filename{STR(MP3Playlists_[State_.mp3PlaylistId()].id + "/" + f.name())};
                    f.close();
                    MP3File_.open(filename.c_str());
                    I2S_.SetGain(State_.volume() * ESP_VOLUME_STEP);
                    MP3_.begin(& MP3File_, & I2S_);
                    LOG("Track %u, file: %s", index, filename.c_str());
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
    static inline AudioGeneratorWAV WAV_;
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
        File f = SD.open("radio/stations.json", FILE_READ);
        if (f) {
            DynamicJsonDocument json{1024};
            int i = 0;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                for (JsonVariant station : json.as<JsonArray>()) {
                    RadioStations_[i++] = station["freq"];
                    LOG("  %s: %u", station["name"].as<char const *>(), station["freq"].as<unsigned>());
                }
            } else {
                LOG("  radio/stations.json file not found");
            }
            f.close();
        } else {
            LOG("  radio/stations.json file not found");
        }
    }

    static void SetRadioFrequency(uint16_t mhzx10) {
        LOG("Radio frequency: %u", mhzx10);
        if (State_.mode() == Mode::Radio) {
            Radio_.setBandFrequency(RADIO_BAND_FM, mhzx10 * 10);            
            State_.setRadioFrequency(mhzx10);
        }
        msg::Send(msg::SetRadioSettings{State_});
    }

    static void SetRadioStation(uint8_t index) {
        LOG("Radio station: %u", index);
        if (State_.mode() == Mode::Radio) {
            Radio_.setBandFrequency(RADIO_BAND_FM, RadioStations_[index] * 10);
            State_.setRadioFrequency(RadioStations_[index]);
            State_.setRadioStation(index);            
            State_.resetControl(State_.radioFrequency() - RADIO_FREQUENCY_OFFSET, RADIO_FREQUENCY_MAX);
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
//@{

    static void SetNightLightEffect(NightLightEffect effect) {
        LOG("Night light effect: %i", static_cast<int>(effect));
        State_.setNightLightEffect(effect);
        msg::Send(msg::SetNightLightSettings{State_});
    }

    static void SetNightLightHue(uint8_t hue) {
        LOG("Night light hue: %u", hue);
        State_.setNightLightHue(hue);
        msg::Send(msg::SetNightLightSettings{State_});
    }
//@}


/** \name Walkie-Talkie
 
    So actually it seems that walkie talkie can interact with telegram bots. Even looks like wavs can be sent and played back.

    https://maakbaas.com/esp8266-iot-framework/logs/https-requests/
    https://core.telegram.org/bots/api#sending-files
 */
//@{

    static void InitializeTelegramBot() {
        File f = SD.open("bot/bot.json", FILE_READ);
        if (f) {
            StaticJsonDocument<1024> json;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                String id = json["id"];
                String token = json["token"];
                BotAdminId_ = json["adminId"].as<int64_t>();
                LOG("Walkie-Talkie:\n  Bot %s\n  Token: %s\n  AdminId: %i", id.c_str(), token.c_str(), BotAdminId_);
                File cf = SD.open("/bot/cert.txt", FILE_READ);
                if (cf && cf.size() < 2048) {
                    String cert = cf.readString();
                    // TODO security is hard, the certificate only works if time is proper...
                    TelegramBot_.initialize(std::move(id), std::move(token) /*, cert.c_str() */);
                    LOG("%s", cert.c_str());
                } else {
                    TelegramBot_.initialize(std::move(id), std::move(token));
                    LOG("No certificate found, telegram bot will be INSECURE!!!");
                }
                cf.close();
            } else {
                LOG("Deserialization error");
            }
            f.close();
        } 
        for (int i = 0; i < 8; ++i)
            BotChannels_[i].clear();
        f = SD.open("bot/chats.json");
        if (f) {
            StaticJsonDocument<1024> json;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                unsigned i = 0;
                for (JsonVariant item : json.as<JsonArray>()) {
                    int64_t id = item["id"];
                    Color color = Color::HTML(item["color"]);
                    LOG("  %u : chat id %lli, color %s - %s",i, id, item["color"].as<char const *>(), item["name"].as<char const *>());
                    BotChannels_[i++] = BotChannel{id, color};
                    if (i >= 8)
                        break;
                }
            } else {
                LOG("Deserialization error");
            }
            f.close();
        }
    }

    /** Returns true if the walkie talkie mode is enabled, i.e. the telegram bot has been configured and WiFi is connected. 
     */
    static bool WalkieTalkieModeEnabled() {
        return (State_.wifiStatus() == WiFiStatus::Connected) && TelegramBot_.isValid();
    }

    static void WalkieTalkieStartRecording() {
        if (!Recording_.begin("/test.wav")) {
            LOG("Unable to open recording target file");
        } else {
            LOG("Recording...");
            Status_.recording = true;
            msg::Send(msg::StartRecording());
        }
    }

    static void WalkieTalkieStopRecording(bool cancel = false) {
        if (Status_.recording) {
            msg::Send(msg::StopRecording{});
            Status_.recording = false;
            Recording_.end();
            LOG("Recording done.");
            //TelegramBot_.sendMessage(BotAdminId_.c_str(), "I am on!");
            File f = SD.open("/test.wav", FILE_READ);
            TelegramBot_.sendAudio(BotAdminId_, f, "test.wav", "audio/wav");
            f.close();
        }
    }

    static void WalkieTalkieCheckUpdates() {
        uint32_t offset = 0;
        StaticJsonDocument<1024> json;
        while (TelegramBot_.getUpdate(json, offset)) {
            if (json["ok"] != true) {
                LOG("Telegram update error (ok: false)");
                return;
            }
            if (! json["result"].is<JsonArray>()) {
                LOG("Telegram update result is not an array");
                return;
            }
            JsonArray const & result = json["result"];
            if (result.size() == 0) {
                LOG("No more updates");
                break;
            }
            JsonObject const & update = result[0];
            offset = update["update_id"].as<uint32_t>() + 1;
            LOG("Update: %u", update);
            if (! update.containsKey("message")) {
                LOG("Update is not a message");
                continue;
            }
            JsonObject const & msg = update["message"];
            // check that it belongs to a valid chat
            uint64_t chatId = msg["chat"]["id"];
            if (msg.containsKey("text")) {
                LOG("text: %s", msg["text"].as<char const *>());
            } else if (msg.containsKey("audio")) {
                JsonObject const & audio = msg["audio"];
                if (audio["mime_type"] != "audio/wav") {
                    LOG("audio of unsupported mimetype %s", audio["mime_type"].as<char const *>());
                } else {
                    LOG("audio message!");
                }
            } else {
                LOG("Unsupported message type");
            }
        }
    }

    struct BotChannel {
        int64_t id;
        Color color;

        BotChannel():
            id{std::numeric_limits<int64_t>::max()} {
        }

        BotChannel(int64_t id, Color const & color):
            id{id},
            color{color} {
        }

        BotChannel & operator = (BotChannel const &) = delete;
        BotChannel & operator = (BotChannel &&) = default;

        void clear() {
            id = std::numeric_limits<int64_t>::max();
        }
    }; // Player::BotChannel

    static inline TelegramBot TelegramBot_;
    static inline int64_t BotAdminId_;
    static inline BotChannel BotChannels_[8];

    static inline WavWriter Recording_;

//@}

/** \name Webserver
 */
//@{
private:

    static void InitializeWiFi() {
        WiFiConnectedHandler_ = WiFi.onStationModeConnected(OnWiFiConnected);
        WiFiIPAssignedHandler_ = WiFi.onStationModeGotIP(OnWiFiIPAssigned);
        WiFiDisconnectedHandler_ = WiFi.onStationModeDisconnected(OnWiFiDisconnected);
    }

    static void WiFiConnect(bool forceAp = false) {
        LOG("WiFi: Scanning networks...");
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        WiFi.scanNetworksAsync([forceAp](int n) {
            LOG("WiFi: Networks found: %i", n);
            File f = SD.open("player/wifi.json", FILE_READ);
            if (f) {
                DynamicJsonDocument json{1024};
                if (deserializeJson(json, f) == DeserializationError::Ok) {
                    if (! forceAp) {
                        for (JsonVariant network : json["networks"].as<JsonArray>()) {
                            for (int i = 0; i < n; ++i) {
                                if (WiFi.SSID(i) == network["ssid"]) {
                                    LOG("WiFi: connecting to %s, rssi: %i, channel: %i", WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
                                    State_.setWiFiStatus(WiFiStatus::Connecting);
                                    msg::Send(msg::SetWiFiStatus{State_});
                                    WiFi.begin(network["ssid"].as<char const *>(), network["password"].as<char const *>());
                                    // so that we do not start the access point just yet
                                    WiFi.scanDelete();
                                    f.close();
                                    return;
                                }
                            }
                        }
                    }
                    // either we are forcing the AP mode, or no suitable networks were found
                    char const * ssid = json["ap"]["ssid"];
                    char const * pass = json["ap"]["pass"];
                    if (ssid == nullptr || ssid[0] == 0) {
                        ssid = DEFAULT_AP_SSID;
                        if (pass == nullptr || pass[0] == 0)
                            pass = DEFAULT_AP_PASSWORD;
                    }
                    LOG("Initializing soft AP, ssid %s, password %p", ssid, pass);
                    LOG("    own ip: 10.0.0.1");
                    LOG("    subnet: 255.255.255.0");
                    IPAddress ip{10,0,0,1};
                    IPAddress subnet{255, 255, 255, 0};
                    WiFi.softAPConfig(ip, ip, subnet);
                    if (!WiFi.softAP(ssid, pass)) 
                        return Error();            
                    State_.setWiFiStatus(WiFiStatus::SoftAP);
                    msg::Send(msg::SetWiFiStatus{State_});
                } else {
                    LOG("");
                }
                f.close();
            } else {
                LOG("WiFi: No player/wifi.json found");
            }
            WiFi.scanDelete();
        });
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
        LOG("WiFi: connected to %s, channel %u", e.ssid.c_str(), e.channel);
    }

    static void OnWiFiIPAssigned(WiFiEventStationModeGotIP const & e) {
        LOG("WiFi: IP assigned: %s, gateway: %s", e.ip.toString().c_str(), e.gw.toString().c_str());
        State_.setWiFiStatus(WiFiStatus::Connected);
        msg::Send(msg::SetWiFiStatus{State_});
    }

    /** TODO what to do when the wifi disconnects, but we did not initiate it? 
     */
    static void OnWiFiDisconnected(WiFiEventStationModeDisconnected const & e) {
        LOG("WiFi: disconnected, reason: %u", e.reason);
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
            PSTR(",\"radioFrequency\":") + State_.radioFrequency() + 
            PSTR(",\"radioStation\":") + State_.radioStation() +
            PSTR(",\"espLoopCount\":") + LoopCount_ +
            PSTR(",\"espMaxLoopTime\":") + MaxLoopTime_ +
            PSTR(",\"freeMem\":") + ESP.getFreeHeap() +
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
        LOG("Cmd: %s", cmd.c_str());
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
        LOG("WebServer: Serving file %s", path.c_str());
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
        LOG("WebServer: Listing directory %s", path.c_str());
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

#endif

#endif // ARCH_ESP8266
