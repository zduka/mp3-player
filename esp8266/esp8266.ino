#include "AudioFileSourceSD.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2SNoDAC.h"
#include <Adafruit_NeoPixel.h>


#include <Wire.h>
#include <radio.h>
#include <RDA5807M.h>

#include "core.h"
#include "wifi_setup.h"

#define CS 16
#define RGB_LED_PIN 5


Adafruit_NeoPixel pixels(8, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

AudioGeneratorMP3 *mp3;
AudioFileSourceSD *file;
AudioOutputI2S *out;


/* ESP8266 PINOUT

         -- RST        up   1 -- TXD0
         -- ADC        up   3 -- I2S_DATA (RXD0)
         -- CH_PC           5 -- SDA
      CS -- 16 up           4 -- SCL
    SCLK -- 14         up   0 --
    MISO -- 12         up   2 -- I2S_WS
    MOSI -- 13       down  15 -- I2S_SCK
         -- VCC           GND --
 
 */

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

#define FIX_BAND     RADIO_BAND_FM    //Radio Band -FM
#define FIX_STATION  9370            //Station Tuned = 93.7 MHz.
#define FIX_VOLUME   5               //Audio Volume Level 5.

RDA5807M radio;    

void setup() {
    Core::Setup();
    //Core::Connect({SSID1,PASSWORD1, SSID2, PASSWORD2});
    //Server.on("/authenticate", server::authenticate);
    //Server.on("/command", server::command);
    //Core::Server.begin();

    pinMode(4, OUTPUT);
    LOG("Setup done.");


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



    // test neopixel
    pixels.begin();
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(64,64,64));
    pixels.show();

    printSD();

    

    file = new AudioFileSourceSD("000/006.mp3");
    out = new AudioOutputI2SNoDAC();
    mp3 = new AudioGeneratorMP3();
    mp3->begin(file, out);    

    //printSD();
}


unsigned long lastMillis = 0;
uint8_t i = 0;

void loop() {
  //Core::Loop();
  // put your main code here, to run repeatedly:
  digitalWrite(4, HIGH);
  if (mp3->isRunning()) {
    if (!mp3->loop()) {
      mp3->stop(); 
      LOG("MP3 done");
    }
  }
  digitalWrite(4, LOW);
  unsigned long t = millis();
  if (t - lastMillis > 1000 / 30) {
    lastMillis = t;
    i += 1;
    pixels.setPixelColor(0, pixels.Color(i,i,i));
    pixels.show();
  }
  
  //delay(5);
  //printSD();
  //delay(900);
}
