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
#include "esp8266/modes.h"
#include "esp8266/wav_mixer.h"

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
 
    TODO make this use less memory
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


}; // ESPSettings


class Player {
    friend class ESPMode;
    friend class MP3Mode;
    friend class RadioMode;
    friend class DiscoMode;
    friend class LightsMode;
    friend class WalkieTalkieMode;
    friend class AlarmMode;
    friend class GreetingMode;
    friend class SyncMode;
    friend class ESPOffMode;

    static inline char const * SETTINGS_FILE = "player/settings.json";
    static inline char const * WIFI_SETTINGS_FILE = "player/wifi.json";

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
        // enable I2C and request the current state and the extended state
        // obtain the first state, without reacting to it
        Wire.begin(I2C_SDA, I2C_SCL);
        Wire.setClock(400000);
        if (! readState()) {
            LOG("Attempt 2:");
            delay(200);
            readState();
        }      
        // get the extended state as well. 
        getExtendedState(ex_);
        ex_.log();
        send(msg::LightsBar{2, 8, adjustBrightness(DEFAULT_COLOR)});
        // initialize the chip and core peripherals
        initializeESP();
        initializeLittleFS();
        initializeWiFi();
        initializeServer();
        send(msg::LightsBar{3, 8, adjustBrightness(DEFAULT_COLOR)});
        initializeSDCard();
        send(msg::LightsBar{4, 8, adjustBrightness(DEFAULT_COLOR)});
        // initialize mode settings from the SD card contents (SD card takes precedence over cached information in extended state)
        mp3Mode_.initialize();
        send(msg::LightsBar{5, 8, adjustBrightness(DEFAULT_COLOR)});
        radioMode_.initialize();
        send(msg::LightsBar{6, 8, adjustBrightness(DEFAULT_COLOR)});
        walkieTalkieMode_.initialize();
        send(msg::LightsBar{7, 8, adjustBrightness(DEFAULT_COLOR)});
        initializeSettings();
        alarmMode_.initialize();
        // store the extended state as it was updated by the SD card
        sendExtendedState(ex_);
        send(msg::SetSettings{
            status_.maxBrightness,
            status_.radioEnabled,
            status_.discoEnabled,
            status_.lightsEnabled,
            status_.syncHour}
        );
        send(msg::LightsBar{8, 8, adjustBrightness(DEFAULT_COLOR)});
        LOG("Free heap: %u", ESP.getFreeHeap());
        ex_.time.log();
        setBusy(false); // just make sure we have no leftovers from resets & stuff
        setIdle(false);
        // set the current mode
        currentMode_ = getModeFor(state_.mode(), state_.musicMode());
        // start with the greering mode, if not resetting the esp
        if (! state_.espReset())
            currentMode_ = & greetingMode_;
        // enter the desired mode
        setMode(currentMode_);
        currentMode_->volumeTurn();
    }


private:

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

    /** Reads the settings from the JSON file stored on the SD card. 
     */
    static void initializeSettings() {
        LOG("Initializing settings...");
        // this should not be needed, but to make sure that at least something is displayed, fill in reasonable defaults here (the bitstruct does not do easy constructor)
        status_.maxBrightness = DEFAULT_BRIGHTNESS;
        status_.timezone = DEFAULT_TIMEZONE;
        status_.maxSpeakerVolume = 15;
        status_.maxHeadphonesVolume = 15;
        status_.syncHour = 3;
        status_.radioEnabled = true;
        status_.discoEnabled = true;
        status_.lightsEnabled = true; 
        StaticJsonDocument<1024> json;
        File f = SD.open(SETTINGS_FILE, FILE_READ);
        if (f && deserializeJson(json, f) == DeserializationError::Ok) {
            status_.maxSpeakerVolume = json["maxSpeakerVolume"];
            status_.maxHeadphonesVolume = json["maxHeadphonesVolume"];
            status_.timezone = json["timezone"];
            status_.maxBrightness = json["maxBrightness"];
            status_.radioEnabled = json["radioEnabled"];
            status_.lightsEnabled = json["lightsEnabled"];
            status_.walkieTalkieEnabled = json["walkieTalkieEnabled"];
            status_.syncHour = json["syncHour"];
            f.close();
        }
    }

    /** \name Main loop and timekeeping. 
     */
    //@{
public:
    static void loop() {
        maxLoopTime_ = std::max(maxLoopTime_, static_cast<uint16_t>(millis() - lastMillis_));
        lastMillis_ = millis();
        bool secondTick = false;
        while (lastMillis_ - lastSecondMillis_ >= 1000) {
            secondTick = true;
            maxLoopTime_ = 0;
            lastSecondMillis_ += 1000;
            ex_.time.secondTick();
        }
        if (status_.irq)
            status_.recording ? updateRecording() : updateState();
        // playback loop
        if (! state_.idle())
            playbackLoop();
        // let the current mode do what it needs to 
        currentMode_->loop(secondTick);
        // if connected to the internet, handle the server & mcast dns (http://mp3-player.local)
        if (state_.wifiStatus() == WiFiStatus::Connected || state_.wifiStatus() == WiFiStatus::AP) {
            server_.handleClient();
            MDNS.update();
        }
        // TODO check time every hour
    }

    static DateTime const & time() {
        return ex_.time;
    }

private:

    static inline uint16_t maxLoopTime_ = 0;
    static inline uint32_t lastMillis_ = 0;
    static inline uint32_t lastSecondMillis_ = 0;

    //@}

    /** \name AVR Communication & Synchronisation
     */
    //@{

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
    static void sendExtendedState(T const & part) {
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

    static void setControlRange(uint16_t value, uint16_t max) {
        LOG("Control range update: %u (max %u)", value, max);
        if (value > max) {
            value = max;
            LOG("  mac value clipped to %u", max);
        }
        state_.setControlValue(value);
        send(msg::SetControlRange{value, max});
    }

    static void sleep() {
        setBusy(true); // disable user controls
        send(msg::Sleep{});
    }

    static bool idle() {
        return state_.idle();
    }

    static void setIdle(bool value) {
        setIdle(value, value ? DEFAULT_IDLE_TIMEOUT : DEFAULT_PLAY_TIMEOUT);
    }

    static void setIdle(bool value, uint8_t timeout) {
        LOG("idle: %u (%u)", value, timeout);
        state_.setIdle(value);
        send(msg::SetIdle{value, timeout});
    }

    static bool busy() {
        return status_.busy;
    }

    static void setBusy(bool value) {
        LOG("busy: %u", value);
        status_.busy = value;
        send(msg::SetESPBusy{value});
    }

    /** Handler for the state update requests.

        These are never done by the interrupt itself, but a flag is raised so that the main loop can handle the actual I2C request when ready. 
     */
    static IRAM_ATTR void avrIrq() {
        status_.irq = true;
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

    /** Requests state update from ATTiny and processes its events.
     */
    static void updateState() {
        State old = state_;
        if (readState()) {
            if (state_.mode() != old.mode()) {
                LOG("mode: %u", static_cast<uint8_t>(state_.mode()));
                setMode(getModeFor(state_.mode(), state_.musicMode()));
            }
            if (state_.controlButtonDown() != old.controlButtonDown())
                if (state_.controlButtonDown()) {
                    LOG("ctrl down");
                    currentMode_->controlDown();
                } else {
                    LOG("ctrl up");
                    currentMode_->controlUp();
                }
            if (state_.controlButtonPress()) {
                LOG("ctrl press");
                currentMode_->controlPress();
            }
            if (state_.controlButtonLongPress()) {
                LOG("ctrl long press");
                currentMode_->controlLongPress();
            }
            if (state_.controlTurn()) {
                LOG("ctrl: %u", state_.controlValue());
                currentMode_->controlTurn();
            }
            if (state_.volumeButtonDown() != old.volumeButtonDown())
                if (state_.volumeButtonDown()) {
                    LOG("vol down");
                    currentMode_->volumeDown();
                } else {
                    LOG("vol up");
                    currentMode_->volumeUp();
                }
            if (state_.volumeButtonPress()) {
                LOG("vol press");
                currentMode_->volumePress();
            }
            if (state_.volumeButtonLongPress()) {
                LOG("vol long press");
                currentMode_->volumeLongPress();
            }
            if (state_.volumeTurn()) {
                LOG("vol: %u", state_.volumeValue());
                // this will check the volume value wrt max volume settings and call the current mode's volume turn event
                updateVolume();
            }
            if (state_.doubleButtonLongPress()) {
                LOG("double long press");
                currentMode_->doubleLongPress();
            }
            // if there is change in headphones, simulate change in volume to cap the volume value if necessary
            if (state_.headphonesConnected() != old.headphonesConnected()) {
                LOG("headphones: %u", state_.headphonesConnected());
                updateVolume();
            }
            // clear the events so that they can be triggered again (ATTiny did so after the transmission as well)
            state_.clearEvents();
        }
    }

    static inline State state_;
    static inline ExtendedState ex_;
    //@}

    /** \name Modes
     */
    //@{

    static inline MP3Mode mp3Mode_;
    static inline RadioMode radioMode_;
    static inline DiscoMode discoMode_;
    static inline LightsMode lightsMode_;
    static inline WalkieTalkieMode walkieTalkieMode_;
    static inline AlarmMode alarmMode_;
    static inline GreetingMode greetingMode_;

    /** The current mode. 
     */
    static inline ESPMode * currentMode_ = nullptr;

    /** Sets the new mode. 
     */
    static void setMode(ESPMode * newMode) {
        MusicMode old = state_.musicMode();
        do {
            if (currentMode_ != nullptr)
                currentMode_->leave(newMode);
            ESPMode * old = currentMode_;
            currentMode_ = newMode;
            newMode = currentMode_->enter(old);
        } while (newMode != currentMode_);
        state_.setMode(newMode->mode);
        send(msg::SetMode{state_.mode(), state_.musicMode()});
        LOG("mode: %u, music: %u", static_cast<uint8_t>(state_.mode()), static_cast<uint8_t>(state_.musicMode()));
    }

    static void setMode(ESPMode & newMode) {
        setMode(& newMode);
    }

    static ESPMode * getModeFor(Mode mode, MusicMode musicMode) {
        switch (mode) {
            case Mode::Music:
                switch (musicMode) {
                    case MusicMode::MP3:
                        return & mp3Mode_;
                    case MusicMode::Radio:
                        return & radioMode_;
                    case MusicMode::Disco:
                        return & discoMode_;
                }
            case Mode::Lights:
                return & lightsMode_;
            case Mode::WalkieTalkie:
                return & walkieTalkieMode_;
            case Mode::Alarm:
                return & alarmMode_;
            // as a fallback, always go to the mp3 mode
            default:
                return & mp3Mode_;
        }
    }
    //@}

    /** \name Playback
     */
    //@{

    /** Starts immediate playback of the selected filename at given volume. 
     
        The volume is expected to be in range 0..15 inclusive. 
     */

    static void playMP3(char const * filename) {
        playMP3(filename, state_.volumeValue());
    }

    static void playMP3(char const * filename, uint8_t volume) {
        LOG("MP3: %s (vol %u)", filename, volume);
        stopPlayback();
        audioFile_.open(filename);
        i2s_.SetGain(static_cast<float>(volume + 1) / 16);
        mp3_.begin(& audioFile_, & i2s_);
    }        

    /** Returns true if the MP3 playback is currently active.
     */
    static bool mp3Playback() {
        return mp3_.isRunning();
    }

    static void stopMP3() {
        if (mp3_.isRunning()) {
            LOG("MP3 stop");
            mp3_.stop();
            i2s_.stop();
            audioFile_.close();
        }
    }

    static void playWAV(char const * filename) {
        playWAV(filename, state_.volumeValue());
    }

    static void playWAV(char const * filename, uint8_t volume) {
        LOG("WAV: %s (vol %u)", filename, volume);
        stopPlayback();
        audioFile_.open(filename);
        i2s_.SetGain(static_cast<float>(volume + 1) / 16);
        wav_.begin(& audioFile_, & i2s_);
    }

    static void playWAV(char const * filename1, char const * filename2, uint8_t volume) {
        LOG("WAV2: %s, %s (vol %u)", filename1, filename2, volume);
        stopPlayback();
        mixer_.open(filename1);
        mixer_.openOverlay(filename2);
        i2s_.SetGain(static_cast<float>(volume + 1) / 16);
        wav_.begin(& mixer_, & i2s_);
    }

    static void stopWAV() {
        if (wav_.isRunning()) {
            LOG("WAV stop");
            wav_.stop();
            i2s_.stop();
            if (audioFile_.isOpen())
                audioFile_.close();
            if (mixer_.isOpen())
                mixer_.close();
        }
    }

    /** Stops all playback done by ESP (if any).
     */
    static void stopPlayback() {
        stopMP3();
        stopWAV();
    }

    static void playbackLoop() {
        if (mp3_.isRunning() && ! mp3_.loop()) {
            stopMP3();
            //mp3_.stop();
            LOG("MP3 playback done");
            currentMode_->playbackFinished();
        } else if (wav_.isRunning() && ! wav_.loop()) {
            stopWAV();
            LOG("WAV playback done");
            currentMode_->playbackFinished();
        }
    }

    /** Adjusts the volume value depending on max volume settings and whether speaker or headphones are connected. 
     */
    static void updateVolume() {
        uint8_t maxVolume = state_.headphonesConnected() ? status_.maxHeadphonesVolume : status_.maxSpeakerVolume;
        if (state_.volumeValue() > maxVolume) {
            state_.setVolumeValue(maxVolume);
            send(msg::SetVolumeRange{state_.volumeValue(), MAX_VOLUME});
            LOG("vol limited to %u", state_.volumeValue());
        }
        currentMode_->volumeTurn();
    }

    static void setI2SVolume(uint8_t volume) {
        i2s_.SetGain(static_cast<float>(volume + 1) / 16);
    }

    static inline AudioGeneratorMP3 mp3_;
    static inline AudioGeneratorWAV wav_;
    static inline AudioOutputI2S i2s_;
    static inline AudioFileSourceSD audioFile_;

    static inline WavMixer mixer_;

    //@}

    /** \name Recording
     */
    //@{

    static void startRecording(char const * filename) {
        if (status_.recording || ! recording_.begin(filename)) {
            ERROR("Unable to open recording target file (or already recording");
        } else {
            LOG("Recording...");
            status_.recording = true;
            send(msg::StartRecording{});
            recordingStart_ = millis();
        }
    }

    static void stopRecording(bool cancel = false) {
        if (status_.recording) {
            LOG("Recording %s, amplitude: %u", cancel ? "cancelled" : "done", recording_.amplitude());
            send(msg::StopRecording{});
            recording_.end();
            status_.recording = false;
            currentMode_->recordingFinished(millis() - recordingStart_, recording_.amplitude());
        }
    }

    static void updateRecording() {
        status_.irq = false;
        size_t n = Wire.requestFrom(AVR_I2C_ADDRESS, 33); // 32 for recorded audio + 1 for state's first byte
        if (n == 33) {
            // add data to the output file
            for (uint8_t i = 0; i < 32; ++i)
                recording_.add(Wire.read());
            // get the first bit of state (we can update our state with it)
            ((uint8_t*)(& state_))[0] = Wire.read();
            // if the volume button has been released or the message is too long, stop recording
            if (! state_.volumeButtonDown() || (millis() - recordingStart_) > MAX_RECORDING_LENGTH)
                stopRecording();
            // otherwise if the control button has been pressed, cancel the recorded sound
            else if (state_.controlButtonDown())
                stopRecording(/* cancel */ false);
        } else {
            ERROR("Incomplete transaction while recording audio, %u bytes received", n);
            // clear the incomplete transaction from the wire buffer
            while (n-- > 0) 
                Wire.read();
        }
    }

    static bool recording() {
        return status_.recording;
    }

    /** The WAV 8kHz recorder to an SD card file
     */
    static inline WavWriter recording_;
    static inline uint32_t recordingStart_;

    //@}

    /** \name WiFi Connection. 
     
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

    /** \name HTTP Server
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
        NTPClient timeClient{ntpUDP, "pool.ntp.org", 3600 * status_.timezone };
        timeClient.begin();
        if (timeClient.forceUpdate()) {
            ex_.time.setFromNTP(timeClient.getEpochTime());
            LOG("NTP time update:");
            ex_.time.log();
            // now that we have obtained the time, send it
            sendExtendedState(ex_.time);
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
            alarmMode_.initialize();
            sendExtendedState(ex_.alarm);
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
        "maxLoopTime:%u"
        "}"),
        ex_.measurements.vcc,
        ex_.measurements.temp,
        ESP.getFreeHeap(),
        state_.charging() ? 1 : 0,
        state_.batteryMode() ? 1 : 0,
        state_.headphonesConnected() ? 1 : 0,
        maxLoopTime_
        );
        server_.send(200, JSON_MIME, buf, len);
    }

    static void httpAlarm() {
        LOG("http alarm");
        char buffer[256];
        int len = alarmMode_.alarmToJson(buffer, sizeof(buffer));
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
                send(msg::LightsBar(upload.totalSize / 1024, upload.contentLength / 1024, adjustBrightness(Color::Blue())));
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


    static Color adjustBrightness(Color const & color) {
        return color.withBrightness(status_.maxBrightness);
    }


    static inline struct {
        bool irq : 1;
        bool busy : 1;

        int timezone : 5; // -16 / + 15
        uint8_t syncHour : 5; // 0..23
        uint8_t maxSpeakerVolume : 4; // 0..15
        uint8_t maxHeadphonesVolume : 4;
        uint8_t maxBrightness : 8; // 0..255

        bool radioEnabled : 1;
        bool discoEnabled : 1;
        bool lightsEnabled : 1;
        bool walkieTalkieEnabled : 1;
        
        bool dualPlayback : 1;
        bool recording : 1;
        bool updateMessages : 1;

    } status_;

}; // Player

// ESPMode --------------------------------------------------------------------------------------------------------

void ESPMode::controlLongPress() {
    Player::setIdle(false);
    MusicMode nextMode = getNextMusicMode(
        Player::state_.mode(),
        Player::state_.musicMode(),
        Player::status_.radioEnabled,
        Player::status_.discoEnabled
    );
    switch (nextMode) {
        case MusicMode::MP3:
            Player::setMode(Player::mp3Mode_);
            break;
        case MusicMode::Radio:
            Player::setMode(Player::radioMode_);
            break;
        case MusicMode::Disco:
            Player::setMode(Player::discoMode_);
            break;
    }
}

void ESPMode::volumeLongPress() {
    if (Player::currentMode_ != & Player::lightsMode_) {
        Player::setIdle(false);
        Player::setMode(Player::lightsMode_);
    } else {
        ESPMode::controlLongPress();
    }
}

void ESPMode::volumeTurn() {
    // volume turn does not reset idle timer as it does not resume playback
    //Player::setIdle(false);
    Player::setI2SVolume(Player::state_.volumeValue());
    Player::send(msg::LightsBar{static_cast<uint16_t>(Player::state_.volumeValue() + 1), 16, adjustBrightness(DEFAULT_COLOR)});
}

void ESPMode::doubleLongPress() {
    if (Player::currentMode_ != & Player::walkieTalkieMode_) {
        Player::setIdle(false);
        Player::setMode(Player::walkieTalkieMode_);
    } else {
        ESPMode::controlLongPress();
    }
}

Color ESPMode::adjustBrightness(Color const & color) {
    return Player::adjustBrightness(color);
}

// MP3Mode -----------------------------------------------------------------------------------------------------------

ESPMode * MP3Mode::enter(ESPMode * prev) {
    Player::state_.setMusicMode(MusicMode::MP3);
    Player::setControlRange(state().trackId, playlists_[state().playlistId].numTracks);
    if (prev != & Player::lightsMode_) {
        Player::setIdle(false);
        setPlaylist(state().playlistId);
    }
    return this;
}

void MP3Mode::leave(ESPMode * next) {
    if (next != & Player::lightsMode_)
        Player::stopMP3();
}

void MP3Mode::playbackFinished() {
    if (state().trackId + 1 < playlists_[state().playlistId].numTracks) {
        setTrack(state().trackId + 1);
    } else {
        Player::stopMP3();
        Player::setIdle(true);
    }
}

void MP3Mode::controlPress() {
    uint8_t i = (state().playlistId + 1) % numPlaylists_;
    setPlaylist(i);
    Player::setIdle(false);
    Player::send(msg::LightsPoint{i, static_cast<uint16_t>(numPlaylists_ - 1), adjustBrightness(MODE_COLOR_MP3)});
}

void MP3Mode::controlTurn() {
    uint8_t trackId = Player::state_.controlValue();
    Player::send(msg::LightsPoint{trackId, static_cast<uint16_t>(playlists_[state().playlistId].numTracks - 1), adjustBrightness(MODE_COLOR_MP3)});
    setTrack(trackId);
    Player::setIdle(false);
}

void MP3Mode::volumePress() {
    if (Player::idle()) {
        Player::setIdle(false);
        if (!Player::mp3Playback())
            setPlaylist(state().playlistId);
    } else {
        Player::setIdle(true);
    }
}

void MP3Mode::initialize() {
    LOG("Initializing MP3 Playlists...");
    File f = SD.open(SETTINGS_FILE, FILE_READ);
    if (f) {
        StaticJsonDocument<1024> json;
        if (deserializeJson(json, f) == DeserializationError::Ok) {
            numPlaylists_ = 0;
            uint8_t id = 0;
            for (JsonVariant playlist : json.as<JsonArray>()) {
                ++id; // so that we start at one
                if (! playlist["enabled"].as<bool>()) // skip disabled playlists
                    continue;
                uint16_t numTracks = initializePlaylist(id);
                if (numTracks > 0) {
                    playlists_[numPlaylists_++] = PlaylistInfo{id, numTracks};
                    LOG("  %u: %s (%u tracks)",id, playlist["name"].as<char const *>(), numTracks);
                } else {
                    LOG("  %u: %s (no tracks found, skipping)", id, playlist["name"].as<char const *>());
                }
            }
        } else {
            LOG("%s is not valid JSON file", SETTINGS_FILE);
        }
        f.close();
    } else {
        LOG("%s not found", SETTINGS_FILE);
    }
}

MP3State & MP3Mode::state() {
    return Player::ex_.mp3;
}

/** Searches the playlist to determine the number of tracks it contains. 
 
    A track is any file ending in `.mp3`. Up to 1023 tracks per playlist are supported in theory, but much smaller numbers should be used. 
 */
uint16_t MP3Mode::initializePlaylist(uint8_t playlistId) {
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

void MP3Mode::setPlaylist(uint8_t index) {
    LOG("Playlist %u (folder %u)", index, playlists_[index].id);
    currentPlaylist_.close();
    char playlistDir[8];
    snprintf_P(playlistDir, sizeof(playlistDir), PSTR("%u"), playlists_[index].id);
    currentPlaylist_ = SD.open(playlistDir);
    state().playlistId = index;
    state().trackId = 0;
    Player::setControlRange(0, playlists_[index].numTracks);
    setTrack(0);
}

void MP3Mode::setTrack(uint16_t index) {
        // pause current playback if any
        Player::stopMP3();
        // determune whether we have to start from the beginning, or can go forward from current track
        uint16_t nextTrack = state().trackId + 1;
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
                    snprintf_P(filename, sizeof(filename), PSTR("%u/%s"), playlists_[state().playlistId].id, f.name());
                    f.close();
                    LOG("Track %u, file: %s", index, filename);
                    Player::playMP3(filename);
                    //audioFile_.open(filename);
                    //i2s_.SetGain(static_cast<float>(state_.volumeValue() + 1) / 16);
                    //mp3_.begin(& audioFile_, & i2s_);
                    state().trackId = index;
                    Player::sendExtendedState(state());
                    return;
                } else {
                    ++nextTrack;
                }
            } 
            f.close();
        }
        LOG("No file found");
}

// RadioMode -------------------------------------------------------------------------------------------------------

ESPMode * RadioMode::enter(ESPMode * prev) {
    Player::state_.setMusicMode(MusicMode::Radio);
    if (prev != & Player::lightsMode_)
        play();
    return this;
}

void RadioMode::leave(ESPMode * next) {
    if (next != & Player::lightsMode_)
        radio_.term();
}

void RadioMode::controlPress() {
    uint8_t i = state().stationId + 1;
    if (i >= numRadioStations_)
        i = 0;
    setRadioStation(i);
    Player::send(msg::LightsPoint{i, static_cast<uint16_t>(numRadioStations_ - 1), adjustBrightness(MODE_COLOR_RADIO)});
}

void RadioMode::controlTurn() {
    if (Player::idle())
        play();

    Player::send(msg::LightsPoint{Player::state_.controlValue(), RADIO_FREQUENCY_MAX - RADIO_FREQUENCY_MIN, adjustBrightness(MODE_COLOR_RADIO)});
    setRadioFrequency(Player::state_.controlValue() + RADIO_FREQUENCY_MIN);
}

void RadioMode::volumePress() {
    if (Player::idle()) {
        Player::setIdle(false);
        play();
    } else {
        Player::setIdle(true);
        pause();
    }
}

void RadioMode::volumeTurn() {
    radio_.setVolume(Player::state_.volumeValue());
    Player::send(msg::LightsBar{static_cast<uint16_t>(Player::state_.volumeValue() + 1), 16, adjustBrightness(DEFAULT_COLOR)});
}

void RadioMode::initialize() {
    LOG("Initializing radio stations...");
    numRadioStations_ = 0;
    File f = SD.open(SETTINGS_FILE, FILE_READ);
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
        LOG("  %s file not found", SETTINGS_FILE);
    }
    // if the current frequency is out of bounds, set it to the frequency of the first station, which just means that when invalid state, default to the first station
    if (state().stationId == RadioState::POWER_ON_STATION_ID) {
        state().stationId = 0;
        state().frequency = radioStations_[0];
    }
}

RadioState & RadioMode::state() {
    return Player::ex_.radio;
}

void RadioMode::play() {
    radio_.init();
    radio_.setMono(forceMono_ || ! Player::state_.headphonesConnected());
    radio_.setVolume(Player::state_.volumeValue());
    setRadioFrequency(state().frequency);
    Player::setControlRange(state().frequency - RADIO_FREQUENCY_MIN, RADIO_FREQUENCY_MAX - RADIO_FREQUENCY_MIN);
    // for reasons unknown to me, the RDA does not always work immediately after power on/wakeup unless we set the frequency again after a while
    delay(100);
    setRadioFrequency(state().frequency);
    Player::setIdle(false);
}

void RadioMode::pause() {
    radio_.term();
    Player::setIdle(true);
}

void RadioMode::setRadioFrequency(uint16_t mhzx10) {
    LOG("Radio frequency: %u", mhzx10);
    state().frequency = mhzx10;
    radio_.setBandFrequency(RADIO_BAND_FM, mhzx10 * 10);            
    Player::sendExtendedState(state());
}

void RadioMode::setRadioStation(uint8_t index) {
    LOG("Radio station: %u", index);
    state().stationId = index;
    state().frequency = radioStations_[index];
    radio_.setBandFrequency(RADIO_BAND_FM, state().frequency * 10);
    Player::sendExtendedState(state());
    Player::setControlRange(state().frequency - RADIO_FREQUENCY_MIN, RADIO_FREQUENCY_MAX - RADIO_FREQUENCY_MIN);
}

// DiscoMode -----------------------------------------------------------------------------------------------------------

ESPMode * DiscoMode::enter(ESPMode * prev) {
    Player::state_.setMusicMode(MusicMode::Disco);
    return this;
}

void DiscoMode::playbackFinished() {
    Player::stopWAV();
    Player::setIdle(true);
}

void DiscoMode::recordingFinished(uint32_t durationMs, uint8_t amplitude) {
    if (durationMs < MIN_WALKIE_TALKIE_RECORDING) {
        LOG("Recording cancelled - too short");
        return;
    }
    Player::setIdle(false);
    Player::playWAV("aha.wav", RECORDING_FILE, 1);
}

void DiscoMode::volumeDown() {
    Player::stopPlayback();
    Player::setIdle(false);
    Player::startRecording(RECORDING_FILE);
}

void DiscoMode::volumeUp() {
    if (Player::recording())
        Player::stopRecording();
}

// LightsMode -------------------------------------------------------------------------------------------------------

ESPMode * LightsMode::enter(ESPMode * prev) {
    Player::setControlRange(state().hue, LightsState::HUE_RAINBOW + 1);
    return this;
}

void LightsMode::controlPress() {
    uint8_t i = (static_cast<uint8_t>(state().effect) + 1) % 8;
    LOG("Lights effect: %u", i);
    state().effect = static_cast<LightsEffect>(i);
    Player::sendExtendedState(state());
    Player::send(msg::LightsPoint{i, 7, adjustBrightness(MODE_COLOR_LIGHTS)});
}

void LightsMode::controlTurn() {
    uint8_t hue = Player::state_.controlValue();
    LOG("Light hue: %u", hue);
    state().hue = hue;
    Player::sendExtendedState(state());
    if (hue == LightsState::HUE_RAINBOW)
        Player::send(msg::LightsColors{
            Color::HSV(0 << 13, 255, Player::status_.maxBrightness),
            Color::HSV(1 << 13, 255, Player::status_.maxBrightness),
            Color::HSV(2 << 13, 255, Player::status_.maxBrightness),
            Color::HSV(3 << 13, 255, Player::status_.maxBrightness),
            Color::HSV(4 << 13, 255, Player::status_.maxBrightness),
            Color::HSV(5 << 13, 255, Player::status_.maxBrightness),
            Color::HSV(6 << 13, 255, Player::status_.maxBrightness),
            Color::HSV(7 << 13, 255, Player::status_.maxBrightness)
        });
    else
        Player::send(msg::LightsBar{8, 8, Color::HSV(state().colorHue(), 255, Player::status_.maxBrightness)});
}

void LightsMode::volumePress() {
    switch (Player::state_.musicMode()) {
        case MusicMode::MP3:
            Player::mp3Mode_.volumePress();
            break;
        case MusicMode::Radio:
            Player::radioMode_.volumePress();
            break;
        case MusicMode::Disco:
            Player::discoMode_.volumePress();
            break;
    }
}

void LightsMode::volumeTurn() {
    switch (Player::state_.musicMode()) {
        case MusicMode::MP3:
            Player::mp3Mode_.volumeTurn();
            break;
        case MusicMode::Radio:
            Player::radioMode_.volumeTurn();
            break;
        case MusicMode::Disco:
            Player::discoMode_.volumeTurn();
            break;
    }
}

LightsState & LightsMode::state() {
    return Player::ex_.lights;
}

// WalkieTalkieMode -------------------------------------------------------------------------------------------------------

ESPMode * WalkieTalkieMode::enter(ESPMode * prev) {
    Player::setControlRange(0, numMessages_);
    Player::connectWiFi();
    Player::setIdle(true);
    return this;
}

void WalkieTalkieMode::leave(ESPMode * next) {
    Player::stopPlayback();
}

void WalkieTalkieMode::loop(bool secondTick) {
    if (enabled()) {
        if (! Player::recording() && secondTick && Player::ex_.time.second() == 0)
            Player::status_.updateMessages = true;
        if (Player::idle() && Player::status_.updateMessages && (Player::state_.wifiStatus() == WiFiStatus::Connected)) {
            Player::status_.updateMessages = false;
            checkBotMessages();
        }
    }
}

void WalkieTalkieMode::playbackFinished() {
    // if we were playing unread message, advance the buffer
    if (! state().isEmpty()) {
        ++state().readId;
        Player::sendExtendedState(state());
    }
    if (playingMessage_ > 0) {
        playMessage(playingMessage_ - 1);
    } else {
        Player::stopWAV();
        Player::setIdle(true);
    }
}

void WalkieTalkieMode::recordingFinished(uint32_t durationMs, uint8_t amplitude) {
    if (durationMs < MIN_WALKIE_TALKIE_RECORDING) {
        LOG("Recording cancelled - too short");
        return;
    }
    File f = SD.open(RECORDING_FILE, FILE_READ);
    Player::setBusy(true);
    bot_.sendAudio(telegramChatId_, f, "audio.wav", "audio/wav", [](uint16_t v, uint16_t m){
        Player::send(msg::LightsBar{v, m, adjustBrightness(MODE_COLOR_WALKIE_TALKIE), 255});    
    });
    Player::send(msg::LightsBar{255, 255, adjustBrightness(MODE_COLOR_WALKIE_TALKIE), 32});
    Player::setBusy(false);
    f.close();
}

void WalkieTalkieMode::controlDown() {
    if (Player::recording())
        Player::stopRecording(/* cancel */ true);
}

void WalkieTalkieMode::controlPress() {
    if (enabled()) {
        Player::setIdle(false);
        play();
    }
}

void WalkieTalkieMode::controlTurn() {
    // if not empty, then the new messages must be played first via normal CTRL press
    if (enabled() && state().isEmpty()) {
        Player::send(msg::LightsPoint{Player::state_.controlValue(), static_cast<uint16_t>(numMessages_) - 1, adjustBrightness(MODE_COLOR_WALKIE_TALKIE)});
        // play from the updated value
        Player::setIdle(false);
        play();
    }
}

void WalkieTalkieMode::volumeDown() {
    if (enabled()) {
        Player::stopPlayback();
        Player::setIdle(false);
        Player::startRecording(RECORDING_FILE);
    }
}

void WalkieTalkieMode::volumeUp() {
    if (enabled() && Player::recording())
        Player::stopRecording();
}

void WalkieTalkieMode::initialize() {
    File f = SD.open(SETTINGS_FILE, FILE_READ);
    if (f) {
        StaticJsonDocument<1024> json;
        if (deserializeJson(json, f) == DeserializationError::Ok) {
            int64_t id = json["id"].as<int64_t>();
            String token = json["token"];
            telegramAdminId_ = json["adminId"].as<int64_t>();
            telegramChatId_ = json["chatId"].as<int64_t>();
            LOG("Walkie-Talkie:\n  Bot %lli\n  Chat %lli\n  Token: %s\n  AdminId: %lli", id, telegramChatId_, token.c_str(), telegramAdminId_);
            File cf = SD.open(CERT_FILE, FILE_READ);
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
    // if this is the initail power on, update the settings accordingly 
    if (state().readId == WalkieTalkieState::POWER_ON_MESSAGE_ID) {
        state().readId = numMessages_ % MAX_WALKIE_TALKIE_MESSAGES;
        state().writeId = numMessages_ % MAX_WALKIE_TALKIE_MESSAGES;
    }
    LOG("read: %u, write: %u", state().readId, state().writeId);
}

WalkieTalkieState & WalkieTalkieMode::state() {
    return Player::ex_.walkieTalkie;
}

bool WalkieTalkieMode::enabled() {
    return Player::status_.walkieTalkieEnabled;
}

void WalkieTalkieMode::startRecording() {
    Player::stopPlayback();
    Player::startRecording(RECORDING_FILE);
}

void WalkieTalkieMode::playMessage(uint8_t offset) {
    playingMessage_ = offset;
    char filename[32];
    uint8_t id = static_cast<uint8_t>(state().writeId - 1 - offset) %         
MAX_WALKIE_TALKIE_MESSAGES;
    LOG("Playing message offset %u, id %u", offset, id);
    snprintf_P(filename, sizeof(filename), PSTR("wt/%u.wav"), id);
    Player::playWAV(filename);
}

void WalkieTalkieMode::play() {
    if (!state().isEmpty())
        playMessage((state().writeId - state().readId - 1) % MAX_WALKIE_TALKIE_MESSAGES);
    else if (numMessages_ > 0) 
        playMessage(static_cast<uint8_t>(Player::state_.controlValue()));
    else 
        return;
}

void WalkieTalkieMode::checkBotMessages() {
    LOG("Checking telegram bot updates...");
    Player::setBusy(true);
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
    Player::setBusy(false);
}

bool WalkieTalkieMode::processBotMessage(JsonObject const & msg, JsonDocument & json) {
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
            if (audio["mime_type"] == Player::WAV_MIME) {
                // if we can't get new message, don't process the message
                if (state().isFull()) {
                    LOG("Buffer full, cannot process");
                    return false;
                }
                LOG("Downloading audio message...");
                char filename[32];
                snprintf_P(filename, sizeof(filename), PSTR("/wt/%u.wav"), state().writeId);
                SD.remove(filename);
                File f = SD.open(filename, FILE_WRITE);
                LOG("filename: %s", filename);
                if (bot_.getFile(audio["file_id"], json, f, [](uint32_t transferred, uint32_t size){
                    // enough for ~60MB
                    Player::send(msg::LightsBar{static_cast<uint16_t>(transferred / 1024), static_cast<uint16_t>(size / 1024), Player::adjustBrightness(MODE_COLOR_WALKIE_TALKIE), 255});    
                })) {
                    LOG("Done, %u bytes.", f.size());
                    state().nextWrite();
                    // update the extended state to flag the info
                    Player::sendExtendedState(state());
                    // update the number of available messages and the control range, if applicable
                    if (numMessages_ < MAX_WALKIE_TALKIE_MESSAGES) {
                        ++numMessages_;
                        if (Player::currentMode_ == this)
                            Player::setControlRange(0, numMessages_);
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

// AlarmMode -------------------------------------------------------------------------------------------------------

ESPMode * AlarmMode::enter(ESPMode * prev) {
    returnMode_ = Player::currentMode_;
    char filename[32];
    getAlarmMP3(filename, sizeof(filename));
    LOG("Alarm!!!");
    Player::playMP3(filename, volume_);
    return this;
}

void AlarmMode::leave(ESPMode * next) {
    Player::stopPlayback();
}

void AlarmMode::initialize() {
    LOG("Setting alarm from SD card");
    File f = SD.open(SETTINGS_FILE, FILE_READ);
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
            Player::ex_.alarm.setHour(h).setMinute(m).enable(enabled, mon, tue, wed, thu, fri, sat, sun);
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

int AlarmMode::alarmToJson(char * buffer, int bufLen) {
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
        alarm().enabled(),
        alarm().hour(),
        alarm().minute(),
        alarm().activeDay(0), 
        alarm().activeDay(1), 
        alarm().activeDay(2), 
        alarm().activeDay(3), 
        alarm().activeDay(4), 
        alarm().activeDay(5), 
        alarm().activeDay(6),
        filename
    );
}

Alarm & AlarmMode::alarm() {
    return Player::ex_.alarm;
}

int AlarmMode::getAlarmMP3(char * buffer, int bufLen) {
    File f = SD.open(SETTINGS_FILE, FILE_READ);
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

void AlarmMode::snooze() {
    LOG("Snooze");
    alarm().snooze();
    LOG("Alarm set to %u:%u", alarm().hour(), alarm().minute());
    Player::sendExtendedState(alarm());
    if (returnMode_ == this) 
        Player::sleep();
    else
        Player::setMode(returnMode_);
}

void AlarmMode::done() {
    LOG("Alarm done");
    // load the alarm from SD card to get rid of any snoozing...
    initialize();
    if (returnMode_ == this) 
        Player::sleep();
    else
        Player::setMode(returnMode_);
}


// GreetingMode -------------------------------------------------------------------------------------------------------

ESPMode * GreetingMode::enter(ESPMode * prev) {
    LOG("Checking greeting...");
    File f = SD.open(SETTINGS_FILE, FILE_READ);
    if (f) {
        StaticJsonDocument<1024> json;
        if (deserializeJson(json, f) == DeserializationError::Ok) {
            for (JsonVariant greeting : json.as<JsonArray>()) {
                bool enabled = greeting["enabled"];
                uint8_t m = greeting["m"];
                uint8_t d = greeting["d"];
                LOG("Greeting date %u/%u, enabled : %u", d, m, enabled ? 1 : 0);
                if (enabled && m == Player::time().month() && d == Player::time().day()) {
                    char const * filename = greeting["file"].as<char const *>();
                    if (filename != nullptr) {
                        LOG("Greeting!!! %s", filename);
                        Player::playMP3(filename);
                        Player::setIdle(false);
                        f.close();
                        return this;
                    }
                }
            }
        }
        f.close();
    }
    return Player::getModeFor(Mode::Music, Player::state_.musicMode());
}

void GreetingMode::dismiss() {
    Player::stopPlayback();
    Player::setMode(Player::getModeFor(Mode::Music, Player::state_.musicMode()));
}

// SyncMode -----------------------------------------------------------------------------------------------------------

ESPMode * SyncMode::enter(ESPMode * prev) {
    LOG("Synchronizing...");
    Player::setBusy(true);
    // first connect to WiFi unless already connected, if in AP mode, terminate AP mode first
    if (Player::state_.wifiStatus() == WiFiStatus::AP)
        Player::disconnectWiFi();
    if (Player::state_.wifiStatus() != WiFiStatus::Connected) {
        Player::connectWiFi();
        uint32_t start = millis();
        while (Player::state_.wifiStatus() == WiFiStatus::Connecting && millis() - start < SYNC_CONNECTION_TIMEOUT) {
            delay(1000);
            Player::setBusy(true); // keep ignoring the user inputs
        };
    }
    // if the connection was successful, proceed with the sync
    if (Player::state_.wifiStatus() == WiFiStatus::Connected) {
        if (! Player::updateNTPTime()) {
            delay(1000);
            Player::updateNTPTime();
        }
        // check walkie talkie messages if walkie talkie enabled
        if (Player::walkieTalkieMode_.enabled())
            Player::walkieTalkieMode_.checkBotMessages();
    }
    // now we are done, let's sleep some more
    LOG("Synchronization done.");
    Player::sleep();
    // wait for the death to come
    while (true) { delay(1000); }
}

// ESPOffMode ---------------------------------------------------------------------------------------------------------

ESPMode * ESPOffMode::enter(ESPMode * prev) {
    LOG("PowerOff");
    Player::sleep();
    // wait for the death to come
    while (true) { delay(1000); }
}

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
