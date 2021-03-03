/** Library for working with neopixels. Comes with the megatinycore and does not require any additional arduino setup. 
 */
#include <tinyNeoPixel_Static.h>



#include "inputs.h"

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
#define CTRL_A 13
#define CTRL_B 10
#define CTRL_BTN 12
#define VOL_A 6
#define VOL_B 5
#define VOL_BTN 7

RotaryEncoder ctrl{CTRL_A, CTRL_B, 255};
RotaryEncoder vol{VOL_A, VOL_B, 15};

Button vol_btn{VOL_BTN};
Button ctrl_btn{CTRL_BTN};

byte pixels[8 * 3];
tinyNeoPixel leds = tinyNeoPixel(2, NEOPIXEL, NEO_GRB + NEO_KHZ800, pixels);

void vol_changed() {
    vol.poll();
}

void vol_btn_changed() {
    vol_btn.poll();
}

void ctrl_changed() {
    ctrl.poll();
}

void ctrl_btn_changed() {
    ctrl_btn.poll();
}


void setup() {
    Serial.swap();
    Serial.begin(9600);
    Serial.println("Hello world");
    // enable 3v3 and 5v 
    pinMode(DCDC_PWR, OUTPUT);
    digitalWrite(DCDC_PWR, HIGH);
    
    pinMode(NEOPIXEL,OUTPUT);
    leds.setPixelColor(0,255,255,255);
    leds.show();
    delay(1000);
    leds.setPixelColor(0,0,0,0); 
    leds.show();                   // LED turns on.
    vol.setInterrupt(vol_changed);
    ctrl.setInterrupt(ctrl_changed);
    vol_btn.setInterrupt(vol_btn_changed);
    ctrl_btn.setInterrupt(ctrl_btn_changed);

}

void loop() {
    if (vol_btn.changed() && vol_btn.pressed())
      vol.setValue(0);
    if (ctrl_btn.changed() && ctrl_btn.pressed()) 
      ctrl.setValue(0);
    if (vol.changed() || ctrl.changed()) {
      leds.setPixelColor(0, vol.value() * 16, ctrl.value(), 0);
      leds.show();
      Serial.print("VOL:");
      Serial.print(vol.value());
      Serial.print(", CTRL:");
      Serial.println(ctrl.value());
      
    }
}
