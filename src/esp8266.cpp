#if (defined ARCH_ESP8266)
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <LittleFS.h>
#include <SPI.h>
#include <SD.h>
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

/** Settings stored on the SD card and used by the ESP alone. 
 
 */  
class ESPSettings {
public:

    uint8_t maxSpeakerVolume = 15;
    uint8_t maxHeadphonesVolume = 15;
    /** Timezone offset in seconds. 
     */
    int32_t timezone = 0;

    uint8_t maxBrightness = DEFAULT_BRIGHTNESS;

    bool radioEnabled = true;
    bool discoEnabled = true;
    bool lightsEnabled = true;
    bool walkieTalkieEnabled = true;

    /** Default hour at which we synchronize
     */
    uint8_t syncHour = 3;

    void fromJSON(JsonDocument const & json) {
        maxSpeakerVolume = json["maxSpeakerVolume"];
        maxHeadphonesVolume = json["maxHeadphonesVolume"];
        timezone = json["timezone"];
        maxBrightness = json["maxBrightness"];
        radioEnabled = json["radioEnabled"];
        lightsEnabled = json["lightsEnabled"];
        walkieTalkieEnabled = json["walkieTalkieEnabled"];
        syncHour = json["syncHour"];
    }

}; // ESPSettings


class Player {
public:

    static inline char const * SETTINGS_FILE = "player/settings.json";
    static inline char const * MP3_SETTINGS_FILE = "player/playlists.json";
    static inline char const * RADIO_SETTINGS_FILE = "player/radio.json";
    static inline char const * WALKIE_TALKIE_SETTINGS_FILE = "player/bot.json";
    static inline char const * WALKIE_TALKIE_CERT_FILE = "player/cert.txt";
    static inline char const * WIFI_SETTINGS_FILE = "player/wifi.json";
    static inline char const * ALARM_SETTINGS_FILE = "player/alarm.json";
    static inline char const * GREETING_SETTINGS_FILE = "player/greeting.json";

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
        // enable I2C and request the current state and the extended state
        // obtain the first state, without reacting to it
        Wire.begin(I2C_SDA, I2C_SCL);
        Wire.setClock(400000);
        if (! readState()) {
            LOG("Attempt 2:");
            delay(200);
            readState();
        }      
        // store if this has been ESP reset, in which case we skip the greeting later on
        bool espReset = state_.espReset();
        // get the extended state as well. 
        getExtendedState(ex_);
        ex_.log();
        send(msg::LightsBar{2, 8, DEFAULT_COLOR.withBrightness(settings_.maxBrightness)});
        // initialize the chip and core peripherals
        initializeESP();
        initializeLittleFS();
        initializeWiFi();
        initializeServer();
        send(msg::LightsBar{3, 8, DEFAULT_COLOR.withBrightness(settings_.maxBrightness)});
        initializeSDCard();
        send(msg::LightsBar{4, 8, DEFAULT_COLOR.withBrightness(settings_.maxBrightness)});
        // initialize mode settings from the SD card contents (SD card takes precedence over cached information in extended state)
        initializePlaylists();
        send(msg::LightsBar{5, 8, DEFAULT_COLOR.withBrightness(settings_.maxBrightness)});
        initializeRadio();
        send(msg::LightsBar{6, 8, DEFAULT_COLOR.withBrightness(settings_.maxBrightness)});
        initializeWalkieTalkie();
        send(msg::LightsBar{7, 8, DEFAULT_COLOR.withBrightness(settings_.maxBrightness)});
        initializeSettings();
        initializeAlarm();
        // if this is the initial state, write basic settings from the mode configuration to cached state
        if (ex_.radio.stationId == 0xff) {
            LOG("Initial power on");
            ex_.radio.stationId = 0;
            ex_.radio.frequency = radioStations_[0];
            ex_.walkieTalkie.readId = numMessages_ % MAX_WALKIE_TALKIE_MESSAGES;
            ex_.walkieTalkie.writeId = numMessages_ % MAX_WALKIE_TALKIE_MESSAGES;
        }
        // store the extended state as it was updated by the SD card
        setExtendedState(ex_);
        send(msg::SetSettings{
            settings_.maxBrightness,
            settings_.radioEnabled,
            settings_.discoEnabled,
            settings_.lightsEnabled,
            settings_.syncHour}
        );
        send(msg::LightsBar{8, 8, DEFAULT_COLOR.withBrightness(settings_.maxBrightness)});
        LOG("Free heap: %u", ESP.getFreeHeap());
        ex_.time.log();
        send(msg::SetESPBusy{false}); // just to be sure we have no leftovers from resets
        switch (state_.mode()) {
            case Mode::Sync:
                sync();
                break;
            default:
                // checkGreeting enters the greeting state if its active, so that we save a few bytes of global state where we do not have to remember the greeting parameters if successful. If there's no greeting we have to set the state ourselves later on. Only check the greeting is this is not a reset situation, in which case proceed immediately to the latest known mode
                if (espReset || ! checkGreeting())
                    setMode(state_.mode());
        }
        // check the volume is within max range
        volumeTurn();
    }

    static void loop() {
        tick();
        if (status_.irq) 
            status_.recording ? record() : updateState();
        if (!state_.idle()) {
            if (mp3_.isRunning()) {
                if (!mp3_.loop()) {
                    mp3_.stop();
                    LOG("MP3 playback done");
                    switch (state_.mode()) {
                        case Mode::Music:
                        case Mode::Lights:
                            playNextTrack();
                            break;
                        case Mode::Alarm:
                            alarmDone();
                            break;
                    }
                }
            } else if (wav_.isRunning()) {
                if (!wav_.loop()) {
                    wav_.stop();
                    LOG("Message playback done");
                    playNextMessage();
                }
            }
        }
        if (state_.wifiStatus() == WiFiStatus::Connected) {
            if (state_.idle()) {
                if (status_.updateMessages) {
                    status_.updateMessages = false;
                    checkBotMessages();
                }
            }
            server_.handleClient();
            MDNS.update();
        }
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
            SD.mkdir("wt");
            return true;
        } else {
            LOG("Error reading from SD card");
            return false;
        }
    }

    /** Initializes the global settings from the SD card.
     
        The settings are split into multiple files so that we can be sure we fit in static JSON buffers. 

         
     */
    static void initializeSettings() {
        StaticJsonDocument<1024> json;
        File f = SD.open(SETTINGS_FILE, FILE_READ);
        if (f && deserializeJson(json, f) == DeserializationError::Ok) {
            settings_.fromJSON(json);
            f.close();
        }
        settings_.syncHour = 21;
    }

    //@}

    /** \name ESP Core 
     */
    //@{
    static void tick() {
        // update the running usage statistics
        uint32_t t = millis();
        ++runningLoopCount_;
        uint16_t delay = t - lastMillis_;
        if (runningMaxLoopTime_ < delay)
            runningMaxLoopTime_ = delay;
        lastMillis_ = t;
        // see if we have a second tick, in which case update the time and utilization counters, then do a second tick
        while (t - previousSecondMillis_ >= 1000) {
            previousSecondMillis_ += 1000;
            loopCount_ = runningLoopCount_;
            maxLoopTime_ = runningMaxLoopTime_;
            runningLoopCount_ = 0;
            runningMaxLoopTime_ = 0;
            secondTick();
        }
    }

    static void secondTick() {
        ex_.time.secondTick();
        if (! status_.recording)
            if (getExtendedState(ex_.measurements))
                send(msg::AVRWatchdogReset{});
            //updateState();
        if (ex_.time.second() == 0 && state_.mode() == Mode::WalkieTalkie && ! status_.recording) {
            if (settings_.walkieTalkieEnabled)
                status_.updateMessages = true;
            updateNTPTime();
        }
        // TODO do more stuff
    }

    /** Synchronizes the player with the rest of the world. 
     
        Attempts to connect to the WiFi and update time from NTP servers. If the walkie-talkie mode is enabled also checks any new telegram messages. The synchronization happens every day at the predefined hour *and* immediately after power on. 
     */
    static void sync() {
        LOG("Synchronizing...");
        send(msg::SetESPBusy{true});
        // first connect to WiFi unless already connected, if in AP mode, terminate AP mode first
        if (state_.wifiStatus() == WiFiStatus::AP)
            disconnectWiFi();
        if (state_.wifiStatus() != WiFiStatus::Connected) {
            connectWiFi();
            uint32_t start = millis();
            while (state_.wifiStatus() == WiFiStatus::Connecting && millis() - start < SYNC_CONNECTION_TIMEOUT) {
                delay(1000);
                send(msg::SetESPBusy{true});
            };
        }
        // if the connection was successful, proceed with the sync
        if (state_.wifiStatus() == WiFiStatus::Connected) {
            if (! updateNTPTime()) {
                delay(1000);
                updateNTPTime();
            }
            // check walkie talkie messages if walkie talkie enabled
            if (settings_.walkieTalkieEnabled)
                checkBotMessages();
        }
        // now we are done, let's sleep some more
        LOG("Synchronization done.");
        send(msg::SetESPBusy{false});
        send(msg::Sleep{});
        // wait for the death to come
        while (true) { delay(100); }
    }

    /** Number of times the loop function gets called in the past second. 
     */
    static inline uint16_t loopCount_ = 0;
    static inline uint16_t maxLoopTime_ = 0;
    static inline uint32_t lastMillis_ = 0;
    static inline uint16_t runningLoopCount_ = 0;
    static inline uint16_t runningMaxLoopTime_ = 0;
    static inline uint32_t previousSecondMillis_ = 0;

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
    static void send(T const & msg, unsigned retries = 1) {
        while (true) {
            Wire.beginTransmission(AVR_I2C_ADDRESS);
            unsigned char const * msgBytes = pointer_cast<unsigned char const *>(& msg);
            Wire.write(msgBytes, sizeof(T));
            uint8_t status = Wire.endTransmission();
            if (status == 0)
                return;
            if (retries-- == 0) {
                ERROR("I2C command failed: %u", status);
                return;
            } else {
                LOG("retrying I2C command, failed: %u", status);
            }
            delay(10); 
        }
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
    static bool getExtendedState(T & part) {
        Wire.beginTransmission(AVR_I2C_ADDRESS);
        Wire.write(static_cast<uint8_t>((uint8_t*)(& part) - (uint8_t *)(& ex_) + sizeof(State)));
        Wire.endTransmission(false); // repeated start
        size_t n = Wire.requestFrom(AVR_I2C_ADDRESS, sizeof(part));
        if (n == sizeof(part)) {
            Wire.readBytes(pointer_cast<uint8_t *>(& part), n);
            return true;
        } else {
            ERROR("Incomplete transaction while reading extended state, %u bytes received", n);
            return false;
        }
    }

    static bool readState() {
        // set IRQ to false before we get the state. It is possibile that the IRQ will be set to true after this but before the state request, thus being cleared on AVR side by the request. This is ok as it would only result in an extra state request afterwards, which would be identical, and therefore no action will be taken
        status_.irq = false;
        size_t n = Wire.requestFrom(AVR_I2C_ADDRESS, sizeof(State));
        if (n == sizeof(State)) {
            Wire.readBytes(pointer_cast<uint8_t*>(& state_), sizeof(State));
            return true;
        } else {
            ERROR("Incomplete transaction while reading state, %u bytes received", n);
            while (n-- > 0) 
                Wire.read();
            return false;
        }
    }

    /** Requests state update from ATTiny. 
     */
    static void updateState() {
        State old = state_;
        if (readState()) {
            // check available events and react accordingly
            if (state_.mode() != old.mode()) {
                switch (state_.mode()) {
                    case Mode::ESPOff:
                        powerOff();
                        break;
                    case Mode::Alarm:
                        // revert the mode first so that we can pause current mode if needed and save it in the setMode function below
                        state_.setMode(old.mode());
                        setMode(Mode::Alarm);
                        break;
                    // this should not happen really
                    default:
                        LOG("Unexpected mode change: %u", static_cast<uint8_t>(state_.mode()));
                        break;
                }
            }
            if (state_.controlTurn()) 
                controlTurn();
            if (state_.controlButtonDown() != old.controlButtonDown())
                state_.controlButtonDown() ? controlDown() : controlUp();
            if (state_.controlButtonPress())
                controlPress();
            if (state_.controlButtonLongPress())
                controlLongPress();
            if (state_.volumeTurn())
                volumeTurn();
            if (state_.volumeButtonDown() != old.volumeButtonDown())
                state_.volumeButtonDown() ? volumeDown() : volumeUp();
            if (state_.volumeButtonPress())
                volumePress();
            if (state_.volumeButtonLongPress())
                volumeLongPress();
            if (state_.doubleButtonLongPress())
                doubleLongPress();
            // if there is change in headphones, simulate change in volume to cap the volume value if necessary
            if (state_.headphonesConnected() != old.headphonesConnected())
                volumeTurn();
            // clear the events so that they can be triggered again (ATTiny did so after the transmission as well)
            state_.clearEvents();
        }
    }

    /** Power off the ESP. 
     */
    static void powerOff() {
        LOG("PowerOff");
        stop();
        send(msg::Sleep{});
        LOG("All done");
        while(true)
            delay(1);
    }

    //@}

    /** \name Controls
     
        Events corresponding directly to user interface events as reported by the ATTiny. Methods in this section translate the raw ui events to actionable items based on the current mode & other context. 
     */
    //@{

    static void controlDown() {
        LOG("Control down");
        switch (state_.mode()) {
            case Mode::WalkieTalkie:
                 if (status_.recording) // can only record if walkie talkie is already enabled
                     stopRecording(/*cancel*/ true);
            default:
                break;
        }
    }

    static void controlUp() {
        LOG("Control up");

    }

    /** Cycles through playlists in mp3 mode, cycles through stations in radio mode, cycles through latest received messages in walkie talkie mode (and plays them) and cycles through light effects for night light mode. 
     */
    static void controlPress() {
        LOG("Control press");
        switch (state_.mode()) {
            case Mode::Music: {
                if (state_.musicMode() == MusicMode::MP3) {
                    uint8_t i = ex_.mp3.playlistId + 1;
                    if (i >= numPlaylists_)
                        i = 0;
                    setPlaylist(i);
                    send(msg::LightsPoint{i, static_cast<uint16_t>(numPlaylists_ - 1), MODE_COLOR_MP3});
                } else {
                    uint8_t i = ex_.radio.stationId + 1;
                    if (i >= numRadioStations_)
                        i = 0;
                    setRadioStation(i);
                    send(msg::LightsPoint{i, static_cast<uint16_t>(numRadioStations_ - 1), MODE_COLOR_RADIO});
                }
                // if not currently playing, start playing now
                if (state_.idle())
                    play();
                break;
            }
            case Mode::Lights: {
                uint8_t i = static_cast<uint8_t>(ex_.lights.effect) + 1;
                if (i == 8)
                    i = 0;
                setLightsEffect(static_cast<LightsEffect>(i));
                send(msg::LightsPoint{i, 7, MODE_COLOR_LIGHTS});
                break;
            }
            case Mode::WalkieTalkie:
                // convert to volume press (play/pause)
                if (settings_.walkieTalkieEnabled)
                    volumePress();
                break;
            case Mode::Alarm:
                snooze();
                break;
            // any button press while in greeting mode moves to the music mode instead
            case Mode::Greeting:
                setMode(Mode::Music);
                break;
            default:
                break;
        }
    }

    /** Cycles through the modes. 
     */
    static void controlLongPress() {
        LOG("Control long press");
        switch (state_.mode()) {
            case Mode::Music: 
                setMusicMode(getNextMusicMode(state_.mode(), state_.musicMode(), settings_.radioEnabled, settings_.discoEnabled));
                // start playing if we were idle
                if (state_.idle())
                    play();
                break;
            case Mode::Alarm:
                alarmDone();
                break;
            // any button press while in greeting mode moves to the music mode instead
            case Mode::Greeting:
                setMode(Mode::Music);
                break;
            default:
                setMode(Mode::Music);
        }
    }

    /** Selects track id in mp3 mode, radio frequency in radio, or changes hue in night lights mode. 
     
        Does nothing in walkie-talkie, can select received messages to play? 
     */
    static void controlTurn() {
        LOG("Control: %u", state_.controlValue());
        switch (state_.mode()) {
            case Mode::Music:
                switch (state_.musicMode()) {
                    case MusicMode::MP3:
                        send(msg::LightsPoint{state_.controlValue(), static_cast<uint16_t>(playlists_[ex_.mp3.playlistId].numTracks - 1), MODE_COLOR_MP3.withBrightness(settings_.maxBrightness)});
                        setTrack(state_.controlValue());
                        break;
                    case MusicMode::Radio:
                        send(msg::LightsPoint{state_.controlValue(), RADIO_FREQUENCY_MAX - RADIO_FREQUENCY_MIN, MODE_COLOR_RADIO.withBrightness(settings_.maxBrightness)});
                        setRadioFrequency(state_.controlValue() + RADIO_FREQUENCY_MIN);
                        break;
                    case MusicMode::Disco:
                        // TODO 
                        break;
                }
                // start playing if we were idle
                if (state_.idle())
                    play();
                break;
            case Mode::WalkieTalkie:
                // if not empty, then the new messages must be played first via normal CTRL press
                if (settings_.walkieTalkieEnabled && ex_.walkieTalkie.isEmpty()) {
                    send(msg::LightsPoint{state_.controlValue(), static_cast<uint16_t>(numMessages_) - 1, MODE_COLOR_WALKIE_TALKIE.withBrightness(settings_.maxBrightness)});
                    // play from the updated value
                    play();
                }
                break;
            case Mode::Lights: {
                setNightLightHue(state_.controlValue());
                if (ex_.lights.hue == LightsState::HUE_RAINBOW)
                    send(msg::LightsColors{
                        Color::HSV(0 << 13, 255, settings_.maxBrightness),
                        Color::HSV(1 << 13, 255, settings_.maxBrightness),
                        Color::HSV(2 << 13, 255, settings_.maxBrightness),
                        Color::HSV(3 << 13, 255, settings_.maxBrightness),
                        Color::HSV(4 << 13, 255, settings_.maxBrightness),
                        Color::HSV(5 << 13, 255, settings_.maxBrightness),
                        Color::HSV(6 << 13, 255, settings_.maxBrightness),
                        Color::HSV(7 << 13, 255, settings_.maxBrightness)
                    });
                else
                    send(msg::LightsBar{8, 8, Color::HSV(ex_.lights.colorHue(), 255, settings_.maxBrightness)});
                break;
            }
            // any button press while in greeting mode moves to the music mode instead
            case Mode::Greeting:
                setMode(Mode::Music);
                break;
            default:
                break;
        }
    }

    /** Starts recording in walkie-talkie mode. 
     */
    static void volumeDown() {
        LOG("Volume down");
        switch (state_.mode()) {
            // TODO recording in disco
            case Mode::WalkieTalkie:
                if (settings_.walkieTalkieEnabled)
                    startRecording();
                break;
            default:
                break;
        }
    }

    static void volumeUp() {
        LOG("Volume up");
        switch (state_.mode()) {
            // TODO recording in disco
            case Mode::WalkieTalkie:
                stopRecording(); // walkie talkie must have been enabled to start the recording, no need to check here
                break;
            default:
                break;
        }
    }

    /** Play/Pause in mp3, radio and lights settings mode. 
     
        Does nothing in walkie-talkie mode as volume key press & hold is used to record messages. 
      */
    static void volumePress() {
        LOG("Volume press");
        switch (state_.mode()) {
            case Mode::Music:
            case Mode::Lights:
                if (state_.musicMode() == MusicMode::Disco)
                    break; // in disco mode, volume pressing does nothing 
            case Mode::WalkieTalkie:
                if (state_.idle())
                    play();
                else
                    pause();
                break;
            case Mode::Alarm:
                snooze();
                break;
            // any button press while in greeting mode moves to the music mode instead
            case Mode::Greeting:
                setMode(Mode::Music);
                break;
            default:
                break;
        }
    }

    static void volumeLongPress() {
        LOG("Volume long press");
        // any button press while in greeting mode moves to the music mode instead
        if (state_.mode() == Mode::Greeting)
            setMode(Mode::Music);
        else if (state_.mode() == Mode::Alarm)
            alarmDone();
        else if (state_.mode() != Mode::WalkieTalkie && state_.musicMode() != MusicMode::Disco)
            setMode(state_.mode() == Mode::Music ? Mode::Lights : Mode::Music);
    }

    /** Changes volume. 
     
        Updates the volume of the underlying music provider (i2s/radio). 
     */
    static void volumeTurn() {
        LOG("Volume: %u", state_.volumeValue());
        // check that the volume is within the settings available
        uint8_t maxVolume = state_.headphonesConnected() ? settings_.maxHeadphonesVolume : settings_.maxSpeakerVolume;
        if (state_.volumeValue() > maxVolume) {
            state_.setVolumeValue(maxVolume);
            send(msg::SetVolumeRange{state_.volumeValue(), MAX_VOLUME});
            LOG("Limited to %u", state_.volumeValue());
        }
        switch (state_.mode()) {
            case Mode::Music:
            case Mode::Lights:
                if (state_.musicMode() == MusicMode::Radio)
                    radioUpdateVolume();
                else // both MP3 and disco use i2s
                    i2sUpdateVolume();
                break;
            case Mode::WalkieTalkie:
            case Mode::Greeting:
                i2sUpdateVolume();
                break;
            default:
                break;
        }
        send(msg::LightsBar{static_cast<uint16_t>(state_.volumeValue() + 1), 16, DEFAULT_COLOR});
    }

    /** Turns wifi on/off, turns the player off. 
     */
    static void doubleLongPress() {
        LOG("Double long press");
        switch (state_.mode()) {
            case Mode::Music:
            case Mode::Lights:
                setMode(Mode::WalkieTalkie);
                break;
            case Mode::WalkieTalkie:
                setMode(Mode::Music);
                break;
            case Mode::Alarm:
                alarmDone();
                break;
            case Mode::Greeting:
                setMode(Mode::Music);
                break;
            default:
                // unreachable
                break;
        }
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

    /** Sets the music mode. 
     */
    static void setMusicMode(MusicMode mode) {
        LOG("Setting music mode %u", mode);
        bool resume = ! state_.idle() && state_.mode() == Mode::Music;
        if (resume)
            stop();
        state_.setMusicMode(mode);
        send(msg::SetMode{state_.mode(), state_.musicMode()});
        if (resume)
            play();
    }    

    static void setMode(Mode mode) {
        LOG("Setting mode %u", mode);
        // if we are coming from walkie talkie, disconnect WiFi too and make sure the playback of messages is stopped
        if (state_.mode() == Mode::WalkieTalkie) {
            walkieTalkieStop();
            if (state_.wifiStatus() != WiFiStatus::Off)
                disconnectWiFi();
        }
        switch (mode) {
            case Mode::Music: {
                // stop if we are moving from the greeting mode so that we can play proper mp3
                if (state_.mode() == Mode::Greeting)
                    stop();
                // if we are going back from lights and already playing, otherwise resume
                bool resume = state_.idle() || (state_.mode() != Mode::Lights);
                state_.setMode(mode);
                send(msg::SetMode{state_.mode(), state_.musicMode()});
                if (resume)
                    play();
                else switch (state_.musicMode()) {
                    case MusicMode::MP3:
                        setControlRange(ex_.mp3.trackId, playlists_[ex_.mp3.playlistId].numTracks);
                        break;
                    case MusicMode::Radio:
                        setControlRange(ex_.radio.frequency - RADIO_FREQUENCY_MIN, RADIO_FREQUENCY_MAX - RADIO_FREQUENCY_MIN);
                        break;
                    case MusicMode::Disco:
                        break;
                }
                break;
            }
            case Mode::Lights:
                state_.setMode(mode);
                send(msg::SetMode{state_.mode(), state_.musicMode()});
                setControlRange(ex_.lights.hue, LightsState::HUE_RAINBOW + 1);
                break;
            case Mode::WalkieTalkie:
                stop(); // stop music if any 
                state_.setMode(mode);
                send(msg::SetMode{state_.mode(), state_.musicMode()});
                setControlRange(0, numMessages_);
                // connect to WiFi, if not already connected
                if (state_.wifiStatus() != WiFiStatus::Connected)
                    connectWiFi();
                // make sure we are ready to record
                status_.recordingReady = true;
                break;
            case Mode::Alarm:
                alarmReturnMode_ = state_.mode();
                alarmResume_ = ! state_.idle();
                stop();
                state_.setMode(mode);
                send(msg::SetMode{state_.mode(), state_.musicMode()});
                play();
                break;
            case Mode::Greeting:
                state_.setMode(mode);
                send(msg::SetMode{state_.mode(), state_.musicMode()});
                play();
                break;
            default:
                break;
        }
    }

    static void play() {
        LOG("Play");
        state_.setIdle(false);
        send(msg::SetIdle{false, timeoutPlay_});
        switch (state_.mode()) {
            case Mode::Music:
            case Mode::Lights:
                switch (state_.musicMode()) {
                    case MusicMode::MP3:
                        // if we are resuming from pause, just setting idle to false above did the trick
                        if (! mp3_.isRunning())
                            mp3Play();
                        break;
                    case MusicMode::Radio:
                        radioPlay();
                        break;
                    case MusicMode::Disco:
                        break;
                }
                break;
            case Mode::WalkieTalkie: 
                if (! wav_.isRunning())
                    walkieTalkiePlay();
                break;
            case Mode::Alarm:
                alarmPlay();
                break;
            // don't do anything - the playback will start later as we don't know the filename at this point
            case Mode::Greeting:
                break;
            default:
                break;
        }
    }

    /** Pauses the playback so that it can be resumed later on. 
     
        At this point it has only meaning for mp3 playback, for all else it is equivalent to a full stop.
     */
    static void pause() {
        LOG("Pause");
        state_.setIdle(true);
        send(msg::SetIdle{true, timeoutIdle_});
        switch (state_.mode()) {
            case Mode::Music:
            case Mode::Lights:
                switch (state_.musicMode()) {
                    case MusicMode::MP3:
                        break;
                    case MusicMode::Radio:
                        radioStop();
                        break;
                    case MusicMode::Disco:
                        break;
                }
                break;
            default:
                break;
        }
    }

    /** Stops the playback. 
     */
    static void stop() {
        LOG("Stop");
        state_.setIdle(true);
        send(msg::SetIdle{true, timeoutIdle_});
        switch (state_.mode()) {
            case Mode::Music:
            case Mode::Lights:
                switch (state_.musicMode()) {
                    case MusicMode::MP3:
                        mp3Stop();
                        break;
                    case MusicMode::Radio:
                        radioStop();
                        break;
                    case MusicMode::Disco:
                        break;
                }
                break;
            case Mode::WalkieTalkie:
                walkieTalkieStop();
            case Mode::Alarm:
            case Mode::Greeting:
                mp3Stop();
                break;
            default:
                break;
        }
    }

    //@}

    /** \name MP3 Mode 
     */
    //@{
    static void initializePlaylists() {
        LOG("Initializing MP3 Playlists...");
        File f = SD.open(MP3_SETTINGS_FILE, FILE_READ);
        if (f) {
            StaticJsonDocument<1024> json;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                numPlaylists_ = 0;
                uint8_t id = 0;
                for (JsonVariant playlist : json.as<JsonArray>()) {
                    ++id; // so that we start at one
                    if (! playlist["enabled"].as<bool>()) // skip disabled playlists
                        continue;
                    uint16_t numTracks = initializePlaylistTracks(id);
                    if (numTracks > 0) {
                        playlists_[numPlaylists_++] = PlaylistInfo{id, numTracks};
                        LOG("  %u: %s (%u tracks)",id, playlist["name"].as<char const *>(), numTracks);
                    } else {
                        LOG("  %u: %s (no tracks found, skipping)", id, playlist["name"].as<char const *>());
                    }
                }
            } else {
                ERROR("%s is not valid JSON file", MP3_SETTINGS_FILE);
            }
            f.close();
        } else {
            LOG("%s not found", MP3_SETTINGS_FILE);
        }
    }

    /** Searches the playlist to determine the number of tracks it contains. 
     
        A track is any file ending in `.mp3`. Up to 1023 tracks per playlist are supported in theory, but much smaller numbers should be used. 
     */
    static uint16_t initializePlaylistTracks(uint8_t playlistId) {
        uint16_t result = 0;
        char playlistFolder[8];
        snprintf_P(playlistFolder, sizeof(playlistFolder), PSTR("%u"), playlistId);
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

    static void mp3Stop() {
        if (mp3_.isRunning()) {
            mp3_.stop();
            i2s_.stop();
            audioFile_.close();
        }
    }

    static void i2sUpdateVolume() {
        if (mp3_.isRunning() || wav_.isRunning()) {
            i2s_.SetGain(static_cast<float>(state_.volumeValue() + 1) / 16);
        }
    }

    /** Sets the given playlist and starts playback from its first track. 
     
        Uses the playlist index, which is the order in which valid playlists in `playtlists.json` were found. 

        Assumes the playlist is valid, which is actually checked by the initializePlaylists. 
     */ 
    static void setPlaylist(uint8_t index) {
        LOG("Playlist %u (folder %u)", index, playlists_[index].id);
        currentPlaylist_.close();
        char playlistDir[8];
        snprintf_P(playlistDir, sizeof(playlistDir), PSTR("%u"), playlists_[index].id);
        currentPlaylist_ = SD.open(playlistDir);
        ex_.mp3.playlistId = index;
        ex_.mp3.trackId = 0;
        setControlRange(0, playlists_[index].numTracks);
        setTrack(0);
    }

    static void setTrack(uint16_t index) {
        // pause current playback if any
        mp3Stop();
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
                    char filename[32]; // this should be plenty for 8.3 filename
                    snprintf_P(filename, sizeof(filename), PSTR("%u/%s"), playlists_[ex_.mp3.playlistId].id, f.name());
                    f.close();
                    audioFile_.open(filename);
                    i2s_.SetGain(static_cast<float>(state_.volumeValue() + 1) / 16);
                    mp3_.begin(& audioFile_, & i2s_);
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
            mp3Stop();
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
    static inline AudioFileSourceSD audioFile_;
    static inline PlaylistInfo playlists_[8];
    static inline uint8_t numPlaylists_ = 0;

    static inline File currentPlaylist_;

    //@}

    /** \name Radio Mode
     
        When enabled, allows the reception of FM radio via a RDA5807M module.
     */
    //@{
    /** Initializes the predefined radio stations from the SD card. 

     */
    static void initializeRadio() {
        LOG("Initializing radio stations...");
        numRadioStations_ = 0;
        File f = SD.open(RADIO_SETTINGS_FILE, FILE_READ);
        if (f) {
            StaticJsonDocument<1024> json;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                for (JsonVariant station : json["stations"].as<JsonArray>()) {
                    radioStations_[numRadioStations_++] = station["freq"];
                    LOG("  %s: %u", station["name"].as<char const *>(), station["freq"].as<unsigned>());
                }
                forceMono_ = json["forceMono"].as<bool>();
                manualTuning_ = json["manualTuning"].as<bool>();
            } else {
                //LOG("  radio/stations.json file not found");
            }
            f.close();
        } else {
            LOG("  %s file not found", RADIO_SETTINGS_FILE);
        }
        // if the current frequency is out of bounds, set it to the frequency of the first station, which just means that when invalid state, default to the first station
        if (ex_.radio.frequency < RADIO_FREQUENCY_MIN || ex_.radio.frequency > RADIO_FREQUENCY_MAX)
            ex_.radio.frequency = radioStations_[0];
    }

    static void radioPlay() {
        radio_.init();
        radio_.setMono(forceMono_ || ! state_.headphonesConnected());
        radio_.setVolume(state_.volumeValue());
        setRadioFrequency(ex_.radio.frequency);
        setControlRange(ex_.radio.frequency - RADIO_FREQUENCY_MIN, RADIO_FREQUENCY_MAX - RADIO_FREQUENCY_MIN);
        // for reasons unknown to me, the RDA does not always work immediately after power on/wakeup unless we set the frequency again after a while
        delay(100);
        setRadioFrequency(ex_.radio.frequency);
    }

    static void radioStop() {
        radio_.term();
    }

    static void radioUpdateVolume() {
        if (state_.mode() == Mode::Music && state_.musicMode() == MusicMode::Radio) {
            radio_.setVolume(state_.volumeValue());
        }
    }

    static void setRadioFrequency(uint16_t mhzx10) {
        LOG("Radio frequency: %u", mhzx10);
        if (state_.mode() == Mode::Music && state_.musicMode() == MusicMode::Radio) {
            ex_.radio.frequency = mhzx10;
            radio_.setBandFrequency(RADIO_BAND_FM, mhzx10 * 10);            
            setExtendedState(ex_.radio);
        }
    }

    static void setRadioStation(uint8_t index) {
        LOG("Radio station: %u", index);
        if (state_.mode() == Mode::Music && state_.musicMode() == MusicMode::Radio) {
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
    static inline bool forceMono_ = false;
    static inline bool manualTuning_ = true;

    //@}

    /** \name Walkie-Talkie Mode
     
        The walkie talkie uses a telegram-bot to support a walkie-talkie like communication between various radios and telegram phones. Each player can be assigned a telegram bot and a chat id that it responds to. Recording a message will send the recorded audio to the specified chat room. When a new voice message in the chat room is detected, it is played.

        

        https://maakbaas.com/esp8266-iot-framework/logs/https-requests/
        https://core.telegram.org/bots/api#sending-files

     */
    //@{

    static void initializeWalkieTalkie() {
        File f = SD.open(WALKIE_TALKIE_SETTINGS_FILE, FILE_READ);
        if (f) {
            StaticJsonDocument<1024> json;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                int64_t id = json["id"].as<int64_t>();
                String token = json["token"];
                telegramAdminId_ = json["adminId"].as<int64_t>();
                telegramChatId_ = json["chatId"].as<int64_t>();
                LOG("Walkie-Talkie:\n  Bot %lli\n  Chat %lli\n  Token: %s\n  AdminId: %lli", id, telegramChatId_, token.c_str(), telegramAdminId_);
                File cf = SD.open(WALKIE_TALKIE_CERT_FILE, FILE_READ);
                if (cf && cf.size() < 2048) {
                    String cert = cf.readString();
                    // TODO security is hard, the certificate only works if time is proper...
                    bot_.initialize(id, std::move(token), nullptr /*, cert.c_str() */);
                } else {
                    bot_.initialize(std::move(id), std::move(token), nullptr);
                    LOG("No certificate found, telegram bot will be INSECURE!!!");
                }
                cf.close();
            } else {
                LOG("Deserialization error");
            }
            f.close();
        }
        // determine the number of messages in the buffer 
        for (numMessages_ = 0; numMessages_ < MAX_WALKIE_TALKIE_MESSAGES; ++numMessages_) {
            char filename[16];
            snprintf_P(filename, sizeof(filename), PSTR("wt/%u.wav"), numMessages_);
            if (! SD.exists(filename))
                break;
        }
        LOG("WT messages found: %u", numMessages_);
    }

    static void startRecording() {
        walkieTalkieStop();
        if (!status_.recordingReady || !recording_.begin("/rec.wav")) {
            LOG("Unable to open recording target file or recording not ready");
            send(msg::LightsFill{Color::Red().withBrightness(settings_.maxBrightness)});
        } else {
            LOG("Recording...");
            status_.recording = true;
            send(msg::StartRecording());
            status_.recordingReady = false;
            messageStart_ = millis();
        }        
    }
        
    static void stopRecording(bool cancel = false) {
        if (status_.recording) {
            // tell avr to return to normal state & stop own recording
            send(msg::StopRecording{});
            recording_.end();
            status_.recording = false;
            cancel = cancel || ((millis() - messageStart_) < MIN_WALKIE_TALKIE_RECORDING);
            if (cancel) {
                LOG("Recording cancelled");
                send(msg::LightsFill{Color::Red().withBrightness(settings_.maxBrightness)});
                status_.recordingReady = true;
            } else {
                LOG("Recording done");
                File f = SD.open("/rec.wav", FILE_READ);
                send(msg::SetESPBusy{true});
                bot_.sendAudio(telegramChatId_, f, "audio.wav", "audio/wav", [](uint16_t v, uint16_t m){
                    send(msg::LightsBar(v, m, MODE_COLOR_WALKIE_TALKIE.withBrightness(settings_.maxBrightness), 255));    
                });
                send(msg::LightsBar(255, 255, MODE_COLOR_WALKIE_TALKIE.withBrightness(settings_.maxBrightness), 32));
                send(msg::SetESPBusy{false});
                f.close();
                status_.recordingReady = true;
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
            // if the volume button has been released or the message is too long, stop recording
            if (! state_.volumeButtonDown() || (millis() - messageStart_) > MAX_WALKIE_TALKIE_RECORDING)
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

    /** Plays stored messages. 
     
        If there are unplayed messages, plays these and then stops. Otherwise pl
     */
    static void walkieTalkiePlay() {
        if (!ex_.walkieTalkie.isEmpty())
            playMessage((ex_.walkieTalkie.writeId - ex_.walkieTalkie.readId - 1) % MAX_WALKIE_TALKIE_MESSAGES);
        else if (numMessages_ > 0) 
            playMessage(static_cast<uint8_t>(state_.controlValue()));
    }

    static void walkieTalkieStop() {
        if (wav_.isRunning()) {
            wav_.stop();
            i2s_.stop();
            audioFile_.close();
        }
    }

    /** Plays the n-th latest message.
     */
    static void playMessage(uint8_t offset) {
        playingMessage_ = offset;
        char filename[32];
        uint8_t id = static_cast<uint8_t>(ex_.walkieTalkie.writeId - 1 - offset) %         
MAX_WALKIE_TALKIE_MESSAGES;
        LOG("Playing message offset %u, id %u", offset, id);
        snprintf_P(filename, sizeof(filename), PSTR("wt/%u.wav"), id);
        audioFile_.open(filename);
        i2s_.SetGain(static_cast<float>(state_.volumeValue() + 1) / 16);
        wav_.begin(& audioFile_, & i2s_);
    }

    static void playNextMessage() {
        // if we were playing unread message, advance the buffer
        if (! ex_.walkieTalkie.isEmpty()) {
            ++ex_.walkieTalkie.readId;
            setExtendedState(ex_.walkieTalkie);
        }
        if (playingMessage_ > 0) {
            playMessage(playingMessage_ - 1);
        } else {
            walkieTalkieStop();
            pause();
        }
    }

    /** Checks the telegram bot messages. 
     
        Start always with update id 0 and then get message by message so that we always fit in the static json document moving the update id accordingly so that the received messages are confirmed and will not repeat next time. 
     */
    static void checkBotMessages() {
        LOG("Checking telegram bot updates...");
        send(msg::SetESPBusy{true});
        int64_t updateId = 0;
        StaticJsonDocument<1024> json;
        while (bot_.getUpdate(json, updateId)) {
            if (json["ok"] != true || ! json["result"].is<JsonArray>()) {
                LOG("Bot update failure");
                break;
            }
            JsonArray const & result = json["result"];
            if (result.size() == 0) {
                LOG("Bot update done.");
                break;
            }
            JsonObject const & update = result[0];
            updateId = update["update_id"].as<int64_t>() + 1;
            LOG("bot update id: %lli", updateId);
            // if the message cannot be currently processed, stop the update so that we can try again            
            if (update.containsKey("message")) {
                if (! processBotMessage(update["message"], json))
                    break; 
            } else if (update.containsKey("channel_post")) {
                if (! processBotMessage(update["channel_post"], json))
                    break;
            } else {
                LOG("Skipping unsuppoerted mesage type");
            }
        }
        send(msg::SetESPBusy{false});
    }

    /** Deal with parsed telegram bot message. 
     
        This can either be text command from the admin, in which case we do as instructed, it can be a text message in the assigned chat, which is printed for debug purposes, or it can be an audio message in general chat which is downloadeded and queued for playing. 

        All other messagfe types are ignored.
     */
    static bool processBotMessage(JsonObject const & msg, JsonDocument & json) {
        int64_t chatId = msg["chat"]["id"];
        if (chatId == telegramAdminId_ && msg.containsKey("text")) {
            if (msg["text"] == "ip") {
                bot_.sendMessage(chatId, WiFi.localIP().toString().c_str());
            } else {
                LOG("unknown command: %s", msg["text"].as<char const *>());
            }
        } else if (chatId == telegramChatId_) {
            if (msg.containsKey("audio")) {
                JsonObject const & audio = msg["audio"];
                if (audio["mime_type"] == WAV_MIME) {
                    // if we can't get new message, don't process the message
                    if (ex_.walkieTalkie.isFull()) {
                        LOG("Buffer full, cannot process");
                        return false;
                    }
                    LOG("Downloading audio message...");
                    char filename[32];
                    snprintf_P(filename, sizeof(filename), PSTR("/wt/%u.wav"), ex_.walkieTalkie.writeId);
                    SD.remove(filename);
                    File f = SD.open(filename, FILE_WRITE);
                    LOG("filename: %s", filename);
                    if (bot_.getFile(audio["file_id"], json, f, [](uint32_t transferred, uint32_t size){
                        // enough for ~60MB
                        send(msg::LightsBar(static_cast<uint16_t>(transferred / 1024), static_cast<uint16_t>(size / 1024), MODE_COLOR_WALKIE_TALKIE.withBrightness(settings_.maxBrightness), 255));    
                    })) {
                        LOG("Done, %u bytes.", f.size());
                        ex_.walkieTalkie.nextWrite();
                        // update the extended state to flag the info
                        setExtendedState(ex_.walkieTalkie);
                        // update the number of available messages and the control range, if applicable
                        if (numMessages_ < MAX_WALKIE_TALKIE_MESSAGES) {
                            ++numMessages_;
                            if (state_.mode() == Mode::WalkieTalkie)
                                setControlRange(0, numMessages_);
                        }
                    } else {
                        LOG("Error");
                    }
                    f.close();
                } else {
                    LOG("Unsupported audio mime-type: %s", audio["mime_type"].as<char const *>());
                }
            } else if (msg.containsKey("text")) {
                LOG("text: %s", msg["text"].as<char const *>());
            }
        } else {
            LOG("Ignoring message from chat %llu", chatId);
        }
        LOG("message processed");
        return true;
    }


    /** The WAV 8kHz recorder to an SD card file
     */
    static inline WavWriter recording_;

    /** The telegram bot that handles the communication. 
     */
    static inline TelegramBot bot_;

    static inline AudioGeneratorWAV wav_;

    static inline int64_t telegramChatId_;
    static inline int64_t telegramAdminId_;
    static inline uint8_t numMessages_;
    static inline uint8_t playingMessage_ = 0;
    /** Start of the message. 
     */
    static inline uint32_t messageStart_;

    //@}

    /** \name Night Lights mode
     */
    //@{

    static void setLightsEffect(LightsEffect effect) {
        LOG("Lights effect: %u", static_cast<uint8_t>(effect));
        ex_.lights.effect = effect;
        setExtendedState(ex_.lights);
    }

    static void setNightLightHue(uint8_t hue) {
        LOG("Light hue: %u", hue);
        ex_.lights.hue = hue;
        setExtendedState(ex_.lights);
    }

    //@}

    /** \name Alarm Clock mode
     
        One alarm can be defined. ESP will wake/up & start playing the selected mp3 file when the time is right. Short press of any of the buttons snoozes the alarm for 5 minutes. Long press stops the alarm (but the alarm will be activated next time it occurs).

        See the `/sd/player/alarm.json` for an example alarm file. To get alarm, use the `alarm` URL. To upload alarm, first upload the `player/alarm.json` file and then call the `cmd?cmd=alarm_upload` command to reload the alarm. 

     */
    //@{

    static void initializeAlarm() {
        LOG("Setting alarm from SD card");
        File f = SD.open(ALARM_SETTINGS_FILE, FILE_READ);
        if (f) {
            StaticJsonDocument<1024> json;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                bool enabled = json["enabled"].as<bool>();
                uint8_t h = json["h"].as<uint8_t>();
                uint8_t m = json["m"].as<uint8_t>();
                bool mon = json["mon"].as<bool>();
                bool tue = json["tue"].as<bool>();
                bool wed = json["wed"].as<bool>();
                bool thu = json["thu"].as<bool>();
                bool fri = json["fri"].as<bool>();
                bool sat = json["sat"].as<bool>();
                bool sun = json["sun"].as<bool>();
                ex_.alarm.setHour(h).setMinute(m).enable(enabled, mon, tue, wed, thu, fri, sat, sun);
                char buf[256];
                alarmToJson(buf, sizeof(buf));
                LOG("Alarm set to: %s", buf);
            } else {
                LOG("Invalid alarm settings");
            }
            f.close();
        } else {
            LOG("No alarm found");
        }
    }

    static int getAlarmMP3(char * buffer, int bufLen) {
        File f = SD.open(ALARM_SETTINGS_FILE, FILE_READ);
        if (f) {
            StaticJsonDocument<1024> json;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                char const * filename = json["file"].as<char const *>();
                if (filename != nullptr)
                    return snprintf_P(buffer, bufLen, PSTR("%s"), filename);
            }
                
            f.close();
        }
        buffer[0] = 0;
        return 0;
    }

    static int alarmToJson(char * buffer, int bufLen) {
        char filename[32];
        getAlarmMP3(filename, sizeof(filename));
        return snprintf_P(buffer, bufLen, PSTR("{"
        "enabled:%u,"
        "h:%u,"
        "m:%u,"
        "mon:%u,"
        "tue:%u,"
        "wed:%u,"
        "thu:%u,"
        "fri:%u,"
        "sat:%u,"
        "sun:%u,"
        "file:\"%s\""
        "}"),
        ex_.alarm.enabled(),
        ex_.alarm.hour(),
        ex_.alarm.minute(),
        ex_.alarm.activeDay(0), 
        ex_.alarm.activeDay(1), 
        ex_.alarm.activeDay(2), 
        ex_.alarm.activeDay(3), 
        ex_.alarm.activeDay(4), 
        ex_.alarm.activeDay(5), 
        ex_.alarm.activeDay(6),
        filename
        );
    }

    static void alarmPlay() {
        char filename[32];
        getAlarmMP3(filename, sizeof(filename));
        audioFile_.open(filename);
        i2s_.SetGain(static_cast<float>(alarmVolume_ + 1) / 16);
        mp3_.begin(& audioFile_, & i2s_);
        LOG("Alarm!!!");
    }

    static void snooze() {
        LOG("Snooze");
        stop();
        ex_.alarm.snooze();
        LOG("Alarm set to %u:%u", ex_.alarm.hour(), ex_.alarm.minute());
        setExtendedState(ex_.alarm);
        if (alarmReturnMode_ != Mode::Alarm) {
            setMode(alarmReturnMode_);
            if (alarmResume_)
                play();
        } else {
            send(msg::Sleep{});
        }
    }

    static void alarmDone() {
        LOG("Alarm done");
        stop();
        // load the alarm from SD card to get rid of any snoozing...
        initializeAlarm();
        if (alarmReturnMode_ != Mode::Alarm) {
            setMode(alarmReturnMode_);
            if (alarmResume_)
                play();
        } else {
            send(msg::Sleep{});
        }
    }

    static inline Mode alarmReturnMode_;
    static inline bool alarmResume_;
    static inline uint8_t alarmVolume_ = DEFAULT_ALARM_VOLUME;

    //@}

    /** \name Birthday Greeting mode
     
        The greeting mode is really simple as no persistent configuration or even ESP memory is needed. At each start we just check whether any of the stored dates apply and if they do play the appropriate greeting. 

        To read/update the greeting information use the sd download and sd upload routines.
     */
    //@{

    /** Checks if a greeting should be played and plays if true. 
     
        Returns true if a greeting is played, false otherwise.
     */
    static bool checkGreeting() {
        LOG("Checking greeting...");
        File f = SD.open(GREETING_SETTINGS_FILE, FILE_READ);
        bool result = false;
        if (f) {
            StaticJsonDocument<1024> json;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
                for (JsonVariant greeting : json.as<JsonArray>()) {
                    bool enabled = greeting["enabled"].as<bool>();
                    uint8_t m = greeting["m"].as<uint8_t>();
                    uint8_t d = greeting["d"].as<uint8_t>();
                    LOG("Greeting date %u/%u, enabled : %u", d, m, enabled ? 1 : 0);
                    if (enabled && m == ex_.time.month() && d == ex_.time.day()) {
                        char const * filename = greeting["file"].as<char const *>();
                        if (filename != nullptr) {
                            result = true;
                            setMode(Mode::Greeting);
                            audioFile_.open(filename);
                            i2s_.SetGain(static_cast<float>(alarmVolume_ + 1) / 16);
                            mp3_.begin(& audioFile_, & i2s_);
                            LOG("Greeting!!!");
                            break;
                        }
                    }
                }
            }
            f.close();
        }
        return result;
    }


    //@}

    /** \name WiFi Connection
     
        Due to pretty limited ESP memory, WiFi is not really compatible with mp3 playback. As such it is only available in the walkie talkie mode and cannot be used in other modes. 
     */
    //@{

    /** WiFi initialization routine. 
     */
    static void initializeWiFi() {
        LOG("Initializing WiFi...");
        // make sure AVR knows wifi is currently off (in case of ESP resets, etc.)
        send(msg::SetWiFiStatus{WiFiStatus::Off});
        wifiConnectedHandler_ = WiFi.onStationModeConnected(onWiFiConnected);
        wifiIPAssignedHandler_ = WiFi.onStationModeGotIP(onWiFiIPAssigned);
        wifiDisconnectedHandler_ = WiFi.onStationModeDisconnected(onWiFiDisconnected);
    }

    /** Connects to the WiFi. 
     
     
     */
    static void connectWiFi() {
        LOG("WiFi: Scanning networks...");
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        state_.setWiFiStatus(WiFiStatus::Connecting);
        WiFi.scanNetworksAsync([](int n) {
            LOG("WiFi: Networks found: %i", n);
            File f = SD.open(WIFI_SETTINGS_FILE, FILE_READ);
            if (f) {
                StaticJsonDocument<1024> json;
                if (deserializeJson(json, f) == DeserializationError::Ok) {
                    for (JsonVariant network : json["networks"].as<JsonArray>()) {
                        for (int i = 0; i < n; ++i) {
                            if (WiFi.SSID(i) == network["ssid"]) {
                                LOG("WiFi: connecting to %s, rssi: %i, channel: %i", WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
                                send(msg::SetWiFiStatus{WiFiStatus::Connecting});
                                WiFi.begin(network["ssid"].as<char const *>(), network["password"].as<char const *>());
                                // so that we do not start the access point just yet
                                WiFi.scanDelete();
                                f.close();
                                return;
                            }
                        }
                    }
                } else {
                    LOG("");
                }
                f.close();
            } else {
                LOG("WiFi: No %s found", WIFI_SETTINGS_FILE);
            }
            WiFi.scanDelete();
            // if no networks were recognized, start AP
            startWiFiAP();
        });
    }

    static void startWiFiAP() {
        File f = SD.open(WIFI_SETTINGS_FILE, FILE_READ);
        if (f) {
            StaticJsonDocument<1024> json;
            if (deserializeJson(json, f) == DeserializationError::Ok) {
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
                if (!WiFi.softAP(ssid, pass)) {
                    // TODO error
                } else {
                    state_.setWiFiStatus(WiFiStatus::AP);
                    send(msg::SetWiFiStatus{WiFiStatus::AP});
                }
            }
            f.close();
        }
    }

    static void disconnectWiFi() {
        LOG("WiFi: Disconnecting");
        WiFi.mode(WIFI_OFF);
        WiFi.forceSleepBegin();
        yield();
        state_.setWiFiStatus(WiFiStatus::Off);
        send(msg::SetWiFiStatus{WiFiStatus::Off});
    }

    static void onWiFiConnected(WiFiEventStationModeConnected const & e) {
        LOG("WiFi: connected to %s, channel %u", e.ssid.c_str(), e.channel);
    }

    static void onWiFiIPAssigned(WiFiEventStationModeGotIP const & e) {
        LOG("WiFi: IP assigned: %s, gateway: %s", e.ip.toString().c_str(), e.gw.toString().c_str());
        state_.setWiFiStatus(WiFiStatus::Connected);
        send(msg::SetWiFiStatus{WiFiStatus::Connected});
    }

    /** TODO what to do when the wifi disconnects, but we did not initiate it? 
     
       reason 2
       reason 3?
       reason 201?

     */
    static void onWiFiDisconnected(WiFiEventStationModeDisconnected const & e) {
        LOG("WiFi: disconnected, reason: %u", e.reason);
        switch (e.reason) {
            case REASON_UNSPECIFIED:
            case REASON_AUTH_EXPIRE:
            case REASON_AUTH_LEAVE:
            case REASON_ASSOC_EXPIRE:
            case REASON_ASSOC_TOOMANY:
            case REASON_NOT_AUTHED:
            case REASON_NOT_ASSOCED:
            case REASON_ASSOC_LEAVE:
            case REASON_ASSOC_NOT_AUTHED:
            case REASON_DISASSOC_PWRCAP_BAD:
            case REASON_DISASSOC_SUPCHAN_BAD:
            case REASON_IE_INVALID:
            case REASON_MIC_FAILURE:
            case REASON_4WAY_HANDSHAKE_TIMEOUT:
            case REASON_GROUP_KEY_UPDATE_TIMEOUT:
            case REASON_IE_IN_4WAY_DIFFERS:
            case REASON_GROUP_CIPHER_INVALID:
            case REASON_PAIRWISE_CIPHER_INVALID:
            case REASON_AKMP_INVALID:
            case REASON_UNSUPP_RSN_IE_VERSION:
            case REASON_INVALID_RSN_IE_CAP:
            case REASON_802_1X_AUTH_FAILED:
            case REASON_CIPHER_SUITE_REJECTED:
            case REASON_BEACON_TIMEOUT:
            case REASON_NO_AP_FOUND:
            case REASON_AUTH_FAIL:
            case REASON_ASSOC_FAIL:
            case REASON_HANDSHAKE_TIMEOUT:
                break;
        }
        if (state_.wifiStatus() != WiFiStatus::Connecting) {
            state_.setWiFiStatus(WiFiStatus::Off);
            send(msg::SetWiFiStatus{WiFiStatus::Off});
        }
    }

    static inline WiFiEventHandler wifiConnectedHandler_;
    static inline WiFiEventHandler wifiIPAssignedHandler_;
    static inline WiFiEventHandler wifiDisconnectedHandler_;

    //@}

    /** \name Web Server & Remote Control
       
     */
    //@{

    static void initializeServer() {
        LOG("Initializing WebServer...");
        if (!MDNS.begin("mp3-player"))
            LOG("  mDNS failed to initialize");
        server_.onNotFound(http404);
        server_.serveStatic("/", LittleFS, "/index.html");
        server_.serveStatic("/app.js", LittleFS, "/app.js");
        server_.serveStatic("/glyphicons.woff2", LittleFS, "/glyphicons.woff2");
        server_.serveStatic("/favicon.ico", LittleFS, "/favicon.ico");
        server_.serveStatic("/bootstrap.min.css", LittleFS, "/bootstrap.min.css");
        server_.serveStatic("/jquery-1.12.4.min.js", LittleFS, "/jquery-1.12.4.min.js");
        server_.serveStatic("/bootstrap.min.js", LittleFS, "/bootstrap.min.js");
        server_.on("/cmd", httpCommand);
        server_.on("/status", httpStatus);
        server_.on("/alarm", httpAlarm);
        server_.on("/sdls", httpSDls);
        server_.on("/sd", httpSD);
        server_.on("/sdUpload", HTTP_POST, httpSDUpload, httpSDUploadHandler);
        server_.begin();
    }

    /** Updates the time from NTP server. 
     
        The time is synchronized with AVR upon a successful update. 
     */
    static bool updateNTPTime() {
        WiFiUDP ntpUDP;
        NTPClient timeClient{ntpUDP, "pool.ntp.org", settings_.timezone};
        timeClient.begin();
        if (timeClient.forceUpdate()) {
            ex_.time.setFromNTP(timeClient.getEpochTime() + settings_.timezone);
            LOG("NTP time update:");
            ex_.time.log();
            // now that we have obtained the time, send it
            setExtendedState(ex_.time);
            return true;
        } else {
            LOG("NTP time updated failed");
            return false;
        }
    }

    static void http404() {
        server_.send(404, "text/json","{ \"response\": 404, \"uri\": \"" + server_.uri() + "\" }");
    }

    static void httpCommand() {
        String const & cmd = server_.arg("cmd");
        if (cmd == "reset") {
            LOG("http reset");
            send(msg::Reset{});
        } else if (cmd == "sleep") {
            LOG("http sleep");
            send(msg::Sleep{});
        } else if (cmd == "reload_alarm") {
            LOG("http reload alarm");
            initializeAlarm();
            setExtendedState(ex_.alarm);
            return httpAlarm();
        } else {
            LOG("http invalid command: %s", cmd.c_str());
            return server_.send(404, GENERIC_MIME, "{response: 404}");
        }
        server_.send(200, GENERIC_MIME, "{response: 200}");
    }

    /** Returns the status of the player.
     */ 
    static void httpStatus() {
        LOG("http status");
        char buf[300];
        int len = snprintf_P(buf, sizeof(buf), PSTR("{"
        "vcc:%u,"
        "temp:%i,"
        "mem:%u,"
        "charging:%u,"
        "batt:%u,"
        "headphones:%u,"
        "loops:%u,"
        "maxLoopTime:%u"
        "}"),
        ex_.measurements.vcc,
        ex_.measurements.temp,
        ESP.getFreeHeap(),
        state_.charging() ? 1 : 0,
        state_.batteryMode() ? 1 : 0,
        state_.headphonesConnected() ? 1 : 0,
        loopCount_,
        maxLoopTime_
        );
        server_.send(200, JSON_MIME, buf, len);
    }

    static void httpAlarm() {
        LOG("http alarm");
        char buffer[256];
        int len = alarmToJson(buffer, sizeof(buffer));
        server_.send(200, JSON_MIME, buffer, len);
    }

    /** Lists a directory on the SD card and returns its contents in a JSON format. 
     */
    static void httpSDls() {
        String const & path = server_.arg("path");
        LOG("WebServer: Listing directory %s", path.c_str());
        File d = SD.open(path.c_str());
        if (!d || !d.isDirectory())
            return http404();
        server_.setContentLength(CONTENT_LENGTH_UNKNOWN);            
        server_.send(200, JSON_MIME, "");
        server_.sendContent("[");
        int n = 0;
        char buf[100];
        while(true) {
            File f = d.openNextFile();
            if (!f)
                break;
            if (n++ > 0)
                server_.sendContent(",");
            if (f.isDirectory())
                server_.sendContent(buf, snprintf_P(buf, sizeof(buf), PSTR("{\"name\":\"%s\", \"size\":\"dir\"}"), f.name()));
            else
                server_.sendContent(buf, snprintf_P(buf, sizeof(buf), PSTR("{\"name\":\"%s\", \"size\":\"%u\"}"), f.name(), f.size()));
        } 
        server_.sendContent("]");
    }

    /** Returns any given file on the SD card. 
     */
    static void httpSD() {
        String const & path = server_.arg("path");
        LOG("http sd: %s", path.c_str());
        File f = SD.open(path.c_str(), FILE_READ);
        if (!f || !f.isFile())
            return http404();
        char const * mime = GENERIC_MIME;
        if (path.endsWith("mp3"))
            mime = MP3_MIME;
        else if (path.endsWith("json"))
            mime = JSON_MIME;
        server_.streamFile(f, mime);
        f.close();
    }

    static void httpSDUpload() {
        server_.send(200);
    }

    static void httpSDUploadHandler() {
        
        HTTPUpload& upload = server_.upload();
        switch (upload.status) {
            case UPLOAD_FILE_START: {
                send(msg::SetESPBusy{true});
                LOG("http file upload start: %s", server_.arg("path").c_str());
                uploadFile_ = SD.open(server_.arg("path").c_str(), "w");
                break;
            }
            case UPLOAD_FILE_WRITE: {
                send(msg::SetESPBusy{true});
                if (uploadFile_)
                    uploadFile_.write(upload.buf, upload.currentSize);
                send(msg::LightsBar(upload.totalSize / 1024, upload.contentLength / 1024, Color::Blue().withBrightness(settings_.maxBrightness)));
                break;
            }
            case UPLOAD_FILE_END: {
                LOG("http upload done.");
                if (uploadFile_) {
                    uploadFile_.close();
                    //server_.send(200, JSON_MIME, "{ response: 200 }");
                } else {
                    server_.send(500, JSON_MIME, "{ response: 500 }");
                }
                send(msg::SetESPBusy{false});
                break;
            }
            case UPLOAD_FILE_ABORTED: {
                LOG("http upload aborted");
                uploadFile_.close();
                SD.remove(server_.arg("path").c_str());
                send(msg::SetESPBusy{false});
                break;
            }
        }
    }

    static inline ESP8266WebServer server_{80};
    static inline File uploadFile_;

    static inline char const * GENERIC_MIME = "text/plain";
    static inline char const * MP3_MIME = "audio/mp3";
    static inline char const * JSON_MIME = "text/json";
    static inline char const * WAV_MIME = "audio/wav";

    //@}






    static inline volatile struct {
        bool irq : 1;
        bool recording : 1;
        /** If true when recording can be done, i.e. the prevcious recording buffer has been sent. False if the buffer is currently being sent. 
         */
        bool recordingReady  : 1;

        /** When true, walkie talkie messages should be updated. 
         */
        bool updateMessages: 1;

    } status_;

    static inline State state_;
    static inline ExtendedState ex_;
    static inline ESPSettings settings_;

    static inline uint8_t timeoutIdle_ = DEFAULT_IDLE_TIMEOUT;
    static inline uint8_t timeoutPlay_ = DEFAULT_PLAY_TIMEOUT;

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

#endif // ARCH_ESP8266
