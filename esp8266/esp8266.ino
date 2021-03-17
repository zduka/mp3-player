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
    void initialize() {
        pinMode(AVR_IRQ, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(AVR_IRQ), AvrIRQ, FALLING);
        Wire.begin();
        // initialize the settings from SD card
        initializeExtraSettings();
        // and now, get the state from AVR and initialize the peripherals as kept in the AVR state
        delay(200);
    }

    /** Enters the radio mode. 
     */
    /*
    void enterRadioMode() {
        LOG("entering radio mode");
        radio_.init();
        radio_.setVolume(state_.volume());
        // we can always simply set radio frequency here because in stations mode, it contains the current station's frequency
        updateRadioFrequency();
        //setRadioFrequency(state_.radioFrequency());
        // update the control dial range depending on the mode used 
        if (state_.radioManualTuning())
            send(Command::EnterRadioMode{state_.radioFrequency() - RADIO_FREQUENCY_OFFSET, RADIO_FREQUENCY_MAX});
        else
            send(Command::EnterRadioMode{state_.radioStation(), 7});
    }
    */

    /*
    void setRadioFrequency(uint16_t frequency) {
        state_.setRadioFrequency(frequency);
        updateRadioFrequency();
        // TODO Send command that updates the frequency on AVR
        }*/

    /*
    void enterMP3Mode() {
        LOG("entering mp3 mode");
        // TODO actually check the number of playlists and the files, the current playlist and file
        mp3File_.open("000/005.mp3");
        i2s_.SetGain(state_.volume() * 0.25);
        mp3_.begin(& mp3File_, & i2s_);
        send(Command::EnterMP3Mode{0, 7});
    }
    */

    /*
    void leaveRadioMode() {
        LOG("leaving radio mode");
        radio_.term();
        } */


    /*
    void leaveMP3Mode() {
        LOG("leaving mp3 mode");
        mp3_.stop();
        i2s_.stop();
        mp3File_.close();
        }*/

    /*
    void setRadioStation(uint16_t i) {
        if (i < 8) {
            LOG("station: " + i);
            currentStation_ = i;
            setRadioFrequency(radioStations_[i].frequency);
        } else {
            LOG("! Invalid station index " + i);
        }
    }
    */

    /*
    void enableManualTuning(bool value) {
        state_->setManualTuning(true);
        send(Command::UpdateRadioState{state_})
        // tell AVR to update control dial value
        } */

    /*
    void togglePlay() {
        LOG("toggle play");
        switch (state_.mode()) {
            case State::Mode::MP3:
                if (mp3_.isRunning()) { 
                    mp3_.stop();
                    send(Command::SetIdle{true});
                }
                break;
            case State::Mode::Radio:
                radio_.setMute(! radio_.getMute());
                send(Command::SetIdle{radio_.getMute()});
                break;
        }
        }*/

    void loop() {
        switch (state_.mode()) {
        case State::Mode::MP3:
            if (mp3_.isRunning()) {
                if (!mp3_.loop()) {
                    mp3_.stop();
                    LOG("MP3 done");
                }
            }
            break;
        }
        if (stateUpdate_)
            getStateUpdate();
    }


    void setMode(State::Mode mode) {
        if (mode != state_.mode()) {
            leaveMode();
            send(Command::SetMode{mode});
        }
        state_.setMode(mode);
        switch (mode) {
            case State::Mode::MP3:
                LOG("mode: mp3");
                // TODO actually check the number of playlists and the files, the current playlist and file
                mp3File_.open("000/005.mp3");
                i2s_.SetGain(state_.audioVolume() * 0.25);
                mp3_.begin(& mp3File_, & i2s_);
                break;
            case State::Mode::Radio:
                LOG("mode: radio");
                radio_.init();
                radio_.setVolume(state_.audioVolume());
                setRadioFrequency(state_.radioFrequency());
                setRadioManualTuning(state_.radioManualTuning());
                break;
        }
    }

    void play() {
        LOG("play");
        switch (state_.mode()) {
            case State::Mode::MP3:
                // TODO
                break;
            case State::Mode::Radio:
                radio_.init();
                radio_.setVolume(state_.audioVolume());
                setRadioFrequency(state_.radioFrequency());
                break;
        }
        if (state_.idle()) {
            state_.setIdle(false);
            send(Command::SetIdle{false});
        }
    }

    void stop() {
        LOG("stop");
        switch (state_.mode()) {
            case State::Mode::MP3:
                // TODO
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

    /** Sets the volume. 

        If the volume is different than current state also changes the state and informs AVR.
     */
    void setAudioVolume(uint16_t volume) {
        volume = (volume > 15) ? 15 : volume;
        LOG("volume: " + volume);
        switch (state_.mode()) {
            case State::Mode::MP3:
                i2s_.SetGain(volume * 0.25);
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
        LOG("radio manual tuning: " + value);
        if (state_.mode() == State::Mode::Radio) {
            if (value) {
                send(Command::SetControl{state_.radioFrequency() - RADIO_FREQUENCY_OFFSET, RADIO_FREQUENCY_MAX});
            } else {
                send(Command::SetControl{state_.radioStation(), 7});
                setRadioStation(state_.radioStation());
            }
        }
        if (state_.radioManualTuning() != value) {
            state_.setRadioManualTuning(value);
            send(Command::SetRadioState{state_});
        }
    }

private:

    void initializeExtraSettings() {
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

    void getStateUpdate() {
        size_t n = Wire.requestFrom(AVR_I2C_ADDRESS,sizeof(State));
        if (n == sizeof(State)) {
            State old{state_};
            Wire.readBytes(pointer_cast<uint8_t*>(& state_), n);
            LOG("I2C state received");
            updateState(old);
        } else {
            LOG("I2C status corruption: " + n);
        }
    }
    

    /** Updates the peripherals according to the state changes. 
     */
    void updateState(State & old) {
        // first react to events, since these will be cleared at the end, there is no need to check them against the old state
        // short press of volume button starts / stops the music 
        if (state_.volumePress())
            state_.idle() ? play() : stop();

        if (state_.volumeLongPress())
            setAudioLights(! state_.audioLights());

        // control press changes playlist in mp3 mode and toggles manual and station tuning in radio
        if (state_.controlPress()) {
            switch (state_.mode()) {
                case State::Mode::MP3:
                    // TODO
                    break;
                case State::Mode::Radio:
                    setRadioManualTuning(!state_.radioManualTuning());
                    break;
            }
        }

        // if volume has changed, update the volume
        if (old.audioVolume() != state_.audioVolume())
            setAudioVolume(state_.audioVolume());
        
        // if control has changed, determine what is the control's function and update accordingly
        if (old.control() != state_.control()) {
            switch (state_.mode()) {
                case State::Mode::MP3:
                    // update the track id
                    break;
                case State::Mode::Radio:
                    if (state_.radioManualTuning())
                        setRadioFrequency(state_.control() + RADIO_FREQUENCY_OFFSET);
                    else
                        setRadioStation(state_.control());
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
                mp3_.stop();
                i2s_.stop();
                mp3File_.close();
                break;
            case State::Mode::Radio:
                LOG("leaving: radio");
                radio_.term();
                break;
        }
    }

    /** Updates the volume settings to reflect current state. 
     */
    /*
    void updateVolume() {
        LOG("volume: " + state_.volume()); 
        switch (state_.mode()) {
            case State::Mode::Radio:
                radio_.setVolume(state_.volume());
                break;
            case State::Mode::MP3:
                i2s_.SetGain(state_.volume() * 0.25);
                break;
        }
        }*/


    /** \name Radio mode
     */
    //@{

    /** Updates the radio-specific state. 
     */
    /*
    void radioModeUpdateState() {
        // sets either the frequency directly in manual tuning mode, or radio station
        if (state_.controlChange()) {
            if (state_.radioManualTuning())
                setRadioFrequency(state_.control() + RADIO_FREQUENCY_OFFSET);
            else
                setRadioStation(state_.control());
        }
        // toggles the manual tuning on or off
        if (state_.controlPress())
            setRadioManualTuning(! state_.radioManualTuning());
        // volume change just changes the volume of the radio as the control is directly linked to volume state
        if (state_.volumeChange())
            updateVolume();
        // toggles radio play on or off
        if (state_.volumePress())
            if (state_.idle())
                play();
            else
                stop();
        // toggles the audio lights on or off
        if (state_.volumeLongPress())
            setAudioLights(!state_.audioLights());
    }
    */

    /** Tells RDA to tune the frequency in the state. 
     */
    /*void updateRadioFrequency() {
        LOG("frequency: " + state_.radioFrequency());
        radio_.setBandFrequency(RADIO_BAND_FM, state_.radioFrequency() * 10);
        } */

    //@}


    

    /*
    void updateControl() {
        LOG("control: " + state_.control());
        switch (state_.mode()) {
            case State::Mode::Radio:
                if (manualTuning_)
                    setRadioFrequency(state_.control() + 760);
                else
                    setRadioStation(state_.control());
                break;
            case State::Mode::MP3:
                break;
        }
    }
    */
    


    /*
    void updateStatus() {
        if (state_.controlLongPress()) {
            switchMode();
        }
        switch (state_.mode()) {
            case State::Mode::Radio:
                radioModeUpdateState();
                break;
        }
        
        state_.clearEvents();
        
        //if (state_.volumeChange())
        //    updateVolume();
        //if (state_.volumePress())
        //    togglePlay();
        //if (state_.volumeLongPress()) {
        //    LOG("toggle audio lights");
        //    send(Command::SetAudioLights{! state_.audioLights()});
        //}
        
        //if (state_.controlChange())
        //    updateControl();
        / *
        if (state_.controlPress()) {
            switch (state_.mode()) {
                case State::Mode::Radio: {
                    state_.setRadioManualTuning(! state_.radioManualTuning());
                    LOG("manual tuning: " + state_.radioManualTuning());
                    if (state_.manualTuning()) {
                        send(Command::SetControl{state_.radioFrequency() - RADIO_FREQUENCY_OFFSET, RADIO_FREQUENCY_MAX});
                    } else {
                        send(Command::SetControl{0, 7});
                        setRadioStation(0);
                    }
                    break;
                }
            }
        }
        if (state_.controlLongPress())
            switchMode();

        state_.clearEvents();
        * /
    }
    */

    /*
    void switchMode() {
        switch (state_.mode()) {
        case State::Mode::MP3:
            leaveMP3Mode();
            enterRadioMode();
            break;
        case State::Mode::Radio:
            leaveRadioMode();
            enterMP3Mode();
            break;
        default:
            enterMP3Mode();
        }
        }*/

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
    
    /** \name Radio & settings. 

        These are extra settings 
     */
    //@{
    RDA5807M radio_;
    //uint16_t radioFrequency_ = 0;
    RadioStation radioStations_[8];
    //uint8_t currentStation_ = 0;
    //bool manualTuning_ = false;
    //@}

    /** \name MP3 & Settings
     */
    //@{
    AudioGeneratorMP3 mp3_;
    AudioOutputI2SNoDAC i2s_;
    AudioFileSourceSD mp3File_;
    
    //@}

    static ICACHE_RAM_ATTR void AvrIRQ();

    static double Gain_[16];

    
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
    player.initialize();
    player.setMode(State::Mode::Radio);
    printSD();
    
    //Core::Connect({SSID1,PASSWORD1, SSID2, PASSWORD2});
    //Server.on("/authenticate", server::authenticate);
    //Server.on("/command", server::command);
    //Core::Server.begin();

    //    pinMode(4, OUTPUT);
    //LOG("Setup done.");

    


    /*
  delay(200);
  radio.init();
  radio.debugEnable();
  radio.setBandFrequency(FIX_BAND, FIX_STATION);
  radio.setVolume(FIX_VOLUME);
  radio.setMono(false);
  radio.setMute(false);
  delay(500);
  char s[12];
  radio.formatFrequency(s, sizeof(s));
  Serial.print("Station:"); 
  Serial.println(s);
  
  Serial.print("Radio:"); 
  radio.debugRadioInfo();
  
  Serial.print("Audio:"); 
  radio.debugAudioInfo();




    printSD();

    


    //printSD();
    */
    //Core::DeepSleep();
}

void loop() {
    player.loop();
  //Core::Loop();
  // put your main code here, to run repeatedly:
  //digitalWrite(4, HIGH);
  //digitalWrite(4, LOW);
  /*
  unsigned long t = millis();
  if (t - lastMillis > 1000 / 30) {
    lastMillis = t;
    i += 1;
    pixels.setPixelColor(0, pixels.Color(i,i,i));
    pixels.show();
  }*/
  
  //delay(5);
  //printSD();
  //delay(900);
}
