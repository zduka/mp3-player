#include "AudioFileSourceSD.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2SNoDAC.h"
#include <Adafruit_NeoPixel.h>

#include "core.h"
#include "wifi_setup.h"

#define CS 16
#define RGB_LED_PIN 5


Adafruit_NeoPixel pixels(8, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

AudioGeneratorMP3 *mp3;
AudioFileSourceSD *file;
AudioOutputI2S *out;


/* ESP8266 PINOUT

         RST        up   1 (TXD0)
         ADC        up   3 (RXD0) (I2S SD)
       CH_PC             5
   (WAKE) 16 up          4
   (SCLK) 14        up   0
   (MISO) 12        up   2 (I2S WS)
   (MOSI) 13      down  15 (I2S SCK)
         VCC           GND
 
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
void setup() {
    Core::Setup();
    Core::Connect({SSID1,PASSWORD1, SSID2, PASSWORD2});
    //Server.on("/authenticate", server::authenticate);
    //Server.on("/command", server::command);
    Core::Server.begin();

    pinMode(4, OUTPUT);
    LOG("Setup done.");

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
