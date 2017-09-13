#include "microBees.h"

WireDuino* wire;
void setup() {
  wire = new WireDuino();
  Serial.begin(57600);
}

void loop() {
  wire->loop();
  Serial.print(wire->realPower);
  Serial.println("W");
  delay(1000);
}
