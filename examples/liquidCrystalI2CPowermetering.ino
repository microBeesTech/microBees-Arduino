#include <Wire.h> 
#include "microBees.h"
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
WireDuino* wire;

void setup()
{
  wire = new WireDuino();
	// initialize the LCD
	lcd.begin();
	lcd.backlight();
	lcd.print("Hello, world!");
}

void loop()
{
  wire->loop();
  String powerString = String(wire->realPower);
  powerString+="W";
  delay(1000);
	lcd.print(powerString.c_str());
}
