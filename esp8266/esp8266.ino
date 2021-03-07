#include <Adafruit_NeoPixel.h>


#include <Wire.h>
#include <radio.h>
#include <RDA5807M.h>

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

struct RadioStation {
    uint16_t frequency;
    String name;
};

class Player {
public:
    void initialize() {
        delay(200);
        pinMode(AVR_IRQ, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(AVR_IRQ), AvrIRQ, FALLING);
    }
    
    void enterRadioMode() {
        Serial.println("-- entering radio mode");
        radio_.init();
        radio_.setVolume(15);
        Serial.println("RADIO MODE ENABLED");
    }

    void setRadioFrequency(uint16_t frequency) {
        radio_.setBandFrequency(RADIO_BAND_FM, frequency);
    }

    
private:

    Power power_;
    Audio audio_;
    Clock clock_;

    RDA5807M radio_;
    uint16_t radioFrequency_ = 9370;
    RadioStation radioStations_[8];

    static ICACHE_RAM_ATTR void AvrIRQ();

    
}; // Player

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


Player player;

/** Handler for the avr irq. 
 */
ICACHE_RAM_ATTR void Player::AvrIRQ() {
    Serial.println("IRQ"); 
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
    player.setRadioFrequency(9370);
    
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
