#pragma once

#include <SD.h>

#include <radio.h>
#include <RDA5807M.h>


#include "config.h"
#include "state.h"
#include "messages.h"
#include "telegram_bot.h"

class ESPMode {
public:

    Mode const mode;

    virtual ESPMode * enter(ESPMode * prev) { return this; };

    virtual void leave(ESPMode * next) {};

    virtual void loop(bool secondTick) {};

    virtual void playbackFinished() {};

    virtual void recordingFinished(uint32_t durationMs) {};

    virtual void controlDown() {};
    virtual void controlUp() {};
    virtual void controlPress() {};

    /** Deafult control long press implementation moves back to one of the music modes, or cycles through the music modes if already in music mode. 
     */
    virtual void controlLongPress();
    virtual void controlTurn() {};
    virtual void volumeDown() {};
    virtual void volumeUp() {};
    virtual void volumePress() {};

    /** Default volume long press implementation enters the lights mode. 
     */
    virtual void volumeLongPress();

    /** Sets the I2S volume and shows the lights bar accordingly. 
     */
    virtual void volumeTurn();

    /** Default double long press enters the walkie talkie & WiFi mode. 
     */
    virtual void doubleLongPress();

    static Color adjustBrightness(Color const & color);

protected:
    ESPMode(Mode mode):
        mode{mode} {
    }
}; // ESPMode

class MP3Mode : public ESPMode {
    friend class Player;
    static constexpr char const * SETTINGS_FILE = "player/playlists.json";
public:
    ESPMode * enter(ESPMode * prev) override;
    void leave(ESPMode * next) override;
    void playbackFinished() override;

    void controlPress() override;
    void controlTurn() override;
    void volumePress() override;
    
    void initialize();

    MP3Mode(): ESPMode{Mode::Music} {}

private:

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

    MP3State & state();

    uint16_t initializePlaylist(uint8_t playlistId);

    void setPlaylist(uint8_t index);

    void setTrack(uint16_t index);

    PlaylistInfo playlists_[8];
    uint8_t numPlaylists_ = 0;

    File currentPlaylist_;

}; // MP3Mode 

class RadioMode : public ESPMode {
    friend class Player;
    static constexpr char const * SETTINGS_FILE = "player/radio.json";
public:
    ESPMode * enter(ESPMode * prev) override;
    void leave(ESPMode * next) override;

    void controlPress() override;
    void controlTurn() override; 
    void volumePress() override;
    void volumeTurn() override;

    void initialize();

    RadioMode(): ESPMode{Mode::Music} {}

private: 

    RadioState & state();

    void play();

    void pause();

    void setRadioFrequency(uint16_t mhzx10);

    void setRadioStation(uint8_t index);

    /** NOTE This uses about 500B RAM, none of which we really need as the only relevant radio state is kept in the extended state. This can be saved if we talk to the radio chip directly. 
     */
    RDA5807M radio_;
    uint8_t numRadioStations_ = 0;
    uint16_t radioStations_[8];
    bool forceMono_ = false;
    bool manualTuning_ = true;

}; // RadioMode

class DiscoMode : public ESPMode {
    friend class Player;
public:
    ESPMode * enter(ESPMode * prev) override;

    DiscoMode(): ESPMode{Mode::Music} {}

}; // DiscoMode

class LightsMode : public ESPMode {
    friend class Player;
public:
    ESPMode * enter(ESPMode * prev) override;

    void controlPress() override;
    void controlTurn() override;
    void volumePress() override;
    void volumeTurn() override;

    LightsMode(): ESPMode{Mode::Lights} {}

private:

    LightsState & state();

}; // LightsMode

class WalkieTalkieMode : public ESPMode {
    friend class Player;
    static inline char const * SETTINGS_FILE = "player/bot.json";
    static inline char const * CERT_FILE = "player/cert.txt";
    static inline char const * RECORDING_FILE = "rec.wav";

public:
    ESPMode * enter(ESPMode * prev) override;
    void leave(ESPMode * next) override;

    void loop(bool secondTick) override;

    void playbackFinished() override;

    void recordingFinished(uint32_t durationMs) override;

    void controlDown() override;
    void controlPress() override;
    void controlTurn() override;
    void volumeDown() override;
    void volumeUp() override;
    void volumePress() override {}
    void volumeLongPress() override {}

    void initialize();

    WalkieTalkieMode(): ESPMode{Mode::WalkieTalkie} {}

private:

    WalkieTalkieState & state();

    bool enabled();

    bool recording();

    void startRecording();

    /** Plays the n-th latest message.
     */
    void playMessage(uint8_t offset);

    void play();



    /** Checks the telegram bot messages. 
     
        Start always with update id 0 and then get message by message so that we always fit in the static json document moving the update id accordingly so that the received messages are confirmed and will not repeat next time. 
     */
    void checkBotMessages();

    /** Deal with parsed telegram bot message. 
     
        This can either be text command from the admin, in which case we do as instructed, it can be a text message in the assigned chat, which is printed for debug purposes, or it can be an audio message in general chat which is downloadeded and queued for playing. 

        All other messagfe types are ignored.
     */
    bool processBotMessage(JsonObject const & msg, JsonDocument & json);

    /** The telegram bot that handles the communication. 
     */
    TelegramBot bot_;

    int64_t telegramChatId_;
    int64_t telegramAdminId_;
    uint8_t numMessages_;
    uint8_t playingMessage_ = 0;
    /** Start of the message. 
     */
    uint32_t messageStart_;
    

}; // WalkieTalkieMode

/** Alarm Clock mode
 
    One alarm can be defined. ESP will wake/up & start playing the selected mp3 file when the time is right. Short press of any of the buttons snoozes the alarm for 5 minutes. Long press stops the alarm (but the alarm will be activated next time it occurs).

    See the `/sd/player/alarm.json` for an example alarm file. To get alarm, use the `alarm` URL. To upload alarm, first upload the `player/alarm.json` file and then call the `cmd?cmd=alarm_upload` command to reload the alarm. 

    */
class AlarmMode : public ESPMode {
    friend class Player;
    static constexpr char const * SETTINGS_FILE = "player/alarm.json";
public:
    ESPMode * enter(ESPMode * prev) override;
    void leave(ESPMode * next) override;
    void playbackFinished() override { done(); }
    void controlPress() override { snooze(); }
    void controlLongPress() override { done(); }
    void volumePress() override { snooze(); }
    void volumeLongPress() override { done(); }
    void doubleLongPress() override { done(); }

    void initialize();

    int alarmToJson(char * buffer, int bufLen);

    AlarmMode(): ESPMode{Mode::Alarm} {}

private:

    Alarm & alarm();

    int getAlarmMP3(char * buffer, int bufLen);

    void snooze();

    void done();

    ESPMode * returnMode_;
    uint8_t volume_ = DEFAULT_ALARM_VOLUME;

}; // AlarmMode

class GreetingMode : public ESPMode {
    friend class Player;
public:

    void checkGreeting();

    GreetingMode(): ESPMode{Mode::Greeting} {}
private:
    void dismiss();

}; // GreetingMode


