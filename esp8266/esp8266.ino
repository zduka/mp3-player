#include "core.h"

void setup() {
  Core::Initialize();
  Core::DisableWiFi();
  Core::ScanNetworks();
}

void loop() {
  // put your main code here, to run repeatedly:

}
