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
    void initialize() {
        Wire.begin();
        delay(200);
        pinMode(AVR_IRQ, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(AVR_IRQ), AvrIRQ, FALLING);
        // TODO replace this with actual stations
        radioStations_[0].frequency = 937; // city
        radioStations_[1].frequency = 1050; // vltava
        radioStations_[2].frequency = 1025; // frekvence 1
        radioStations_[3].frequency = 1007; // CRO
        radioStations_[4].frequency = 997; // Bonton 
        radioStations_[5].frequency = 987; // Classic
        radioStations_[6].frequency = 972; // Fajn 
        radioStations_[7].frequency = 966; // Impuls
        radioFrequency_ = radioStations_[0].frequency;
    }

    /** Sets the volume. 
     */
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
    }

    void updateControl() {
        LOG("control: " + state_.control());
        switch (state_.mode()) {
            case State::Mode::Radio:
                setRadioStation(state_.control());
                break;
            case State::Mode::MP3:
                break;
        }
    }

    /** Enters the radio mode. 
     */
    void enterRadioMode() {
        LOG("entering radio mode");
        radio_.init();
        radio_.setVolume(state_.volume());
        setRadioFrequency(radioFrequency_);
        send(Command::EnterRadioMode(0, 7));
    }

    void leaveRadioMode() {
        LOG("leaving radio mode");
        radio_.term();
    }

    void enterMP3Mode() {
        LOG("entering mp3 mode");
        mp3File_.open("000/005.mp3");
        i2s_.SetGain(state_.volume() * 0.25);
        mp3_.begin(& mp3File_, & i2s_);
        send(Command::EnterMP3Mode(0, 7));
    }

    void leaveMP3Mode() {
        LOG("leaving mp3 mode");
        mp3_.stop();
        i2s_.stop();
        mp3File_.close();
    }

    void setRadioFrequency(uint16_t frequency) {
        LOG("frequency: " + frequency);
        radioFrequency_ = frequency;
        radio_.setBandFrequency(RADIO_BAND_FM, frequency * 10);
    }

    void setRadioStation(uint16_t i) {
        if (i < 8) {
            LOG("station: " + i);
            currentStation_ = i;
            setRadioFrequency(radioStations_[i].frequency);
        } else {
            LOG("! Invalid station index " + i);
        }
    }

    void enableManualTuning(bool value) {
        manualTuning_ = true;
        // tell AVR to update control dial value
    }

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
        if (state_.irq())
            getStatus();
    }

    
private:

    void getStatus();

    void updateStatus() {
        if (state_.volumeChange())
            updateVolume();
        if (state_.controlChange())
            updateControl();
        if (state_.controlPress())
            switchMode();

        state_.clearEvents();
    }

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
    }

    template<typename T>
    void send(T msg) {
        Wire.beginTransmission(AVR_I2C_ADDRESS);
        Wire.write(T::Id);
        Wire.write(pointer_cast<char *>(& msg), sizeof(T));
        Wire.endTransmission();
    }

    State state_;
    
    /** \name Radio & settings. 
     */
    //@{
    RDA5807M radio_;
    uint16_t radioFrequency_ = 0;
    RadioStation radioStations_[8];
    uint8_t currentStation_ = 0;
    bool manualTuning_ = false;
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

void Player::getStatus() {
    size_t n = Wire.requestFrom(AVR_I2C_ADDRESS,sizeof(State));
    if (n == sizeof(State)) {
        Wire.readBytes(pointer_cast<uint8_t*>(& state_), n);
        LOG("state received: m" + (int)state_.mode() + " c" + state_.control() + " v" + state_.volume());
        updateStatus();
    } else {
        /*            
                      player.state_.clearIrq();
                      irq_ = false;
                      uint8_t * x = pointer_cast<uint8_t*>(& player.state_);
                      while (Wire.available()) {
                      *x = Wire.read();
                      ++x;
                      }
                      if (state.volumeChange())
                      setVolume(state.volume());
                      } else { */
        LOG("I2C status corruption: " + n);
    }
}

/** Handler for the avr irq. 
 */
ICACHE_RAM_ATTR void Player::AvrIRQ() {
    // Serial.println("IRQ");
    player.state_.setIrq();
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
    player.enterRadioMode();
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
