#include <Wire.h>
#include <radio.h>
#include <RDA5807M.h>
#include <AudioFileSourceSD.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2SNoDAC.h>

#include "core.h"
#include "wifi_setup.h"
#include "state.h"


/* ESP8266 PINOUT

         -- RST        up   1 -- TXD0
         -- ADC        up   3 -- I2S_DATA (RXD0)
         -- CH_PC           5 -- SDA
      CS -- 16 up           4 -- SCL
    SCLK -- 14         up   0 -- AVR_IRQ
    MISO -- 12         up   2 -- I2S_WS
    MOSI -- 13       down  15 -- I2S_SCK
         -- VCC           GND --
 
 */

#define CS 16
#define AVR_IRQ 0

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
        Wire.begin();
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
        state_.setMp3TrackId(i);
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






void printSD() {
    if (!SD.begin(CS, SPI_HALF_SPEED)) {
      Serial.println("Initialising failed!");
      return;
    }    
    File root;
    root = SD.open("/");
    root.rewindDirectory();
    printDirectory(root, 0); //Display the card contents
    root.close();
    Serial.println("\r\nOPEN FILE example completed");  
}


void printDirectory(File dir, int numTabs) {
    int colcnt =0;
    while(true) {
        File entry =  dir.openNextFile();
        if (! entry) {
            // no more files
            break;
        }
        if (numTabs > 0) {
           for (uint8_t i=0; i<=numTabs; i++) {
               Serial.print('\t');
           }
        }
        Serial.print(entry.name());
        if (entry.isDirectory()) {
            Serial.println("/");
            printDirectory(entry, numTabs+1);
        } else {
            // files have sizes, directories do not
            Serial.print("\t");
            Serial.println(entry.size(), DEC);
        }
    }
}

//#define FIX_BAND     RADIO_BAND_FM    //Radio Band -FM
//#define FIX_STATION  9370            //Station Tuned = 93.7 MHz.
//#define FIX_VOLUME   15               //Audio Volume Level 5.

//RDA5807M radio;    

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
