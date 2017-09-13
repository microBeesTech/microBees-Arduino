#include "microBees.h"

WireDuino* wire;
uint8_t target=0;
void setup() {
  wire = new WireDuino();
  Serial.begin(57600);
}

void loop() {
  target = !target;
  wire->setRelay(target);
  delay(1000);
}
