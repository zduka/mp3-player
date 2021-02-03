#include "core.h"

#include "wifi_setup.h"

void setup() {
    Core::Setup();
    Core::Connect({SSID1,PASSWORD1, SSID2, PASSWORD2});
    //Server.on("/authenticate", server::authenticate);
    //Server.on("/command", server::command);
    Core::Server.begin();

    pinMode(2, OUTPUT);
    LOG("Setup done.");
}

void loop() {
  Core::Loop();
  // put your main code here, to run repeatedly:
  digitalWrite(2, HIGH);
  delay(5);
  digitalWrite(2, LOW);
  delay(5);

}
