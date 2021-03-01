/** Library for working with neopixels. Comes with the megatinycore and does not require any additional arduino setup. 
 */
#include <tinyNeoPixel_Static.h>

/** Chip Pinout

               -- VDD   GND --
     VCC_SENSE -- (00) PA4   PA3 (16) --
    HEADPHONES -- (01) PA5   PA2 (15) -- MIC
      DCDC_PWR -- (02) PA6   PA1 (14) -- 
     AUDIO_SRC -- (03) PA7   PA0 (17) -- UPDI
     AUDIO_ADC -- (04) PB5   PC3 (13) -- CTRL_A
         VOL_B -- (05) PB4   PC2 (12) -- CTRL_BTN
         VOL_A -- (06) PB3   PC1 (11) -- NEOPIXEL
       VOL_BTN -- (07) PB2   PC0 (10) -- CTRL_B
           SDA -- (08) PB1   PB0 (09) -- SCL
 */

#define DCDC_PWR 2
#define NEOPIXEL 11

byte pixels[8 * 3];
tinyNeoPixel leds = tinyNeoPixel(2, NEOPIXEL, NEO_GRB + NEO_KHZ800, pixels);


void setup() {
    // enable 3v3 and 5v 
    pinMode(DCDC_PWR, OUTPUT);
    digitalWrite(DCDC_PWR, HIGH);
    
    pinMode(NEOPIXEL,OUTPUT);
    delay(1000);
    leds.setPixelColor(0,0,0,32); // first LED full RED
    leds.show();                   // LED turns on.
    
}

void loop() {

}
