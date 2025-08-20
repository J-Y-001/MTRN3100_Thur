// Test for minimum program size.

#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1

SSD1306AsciiWire oled;
//------------------------------------------------------------------------------
void setup() {
  Wire.begin();
  Wire.setClock(400000L);
  
  oled.begin(&Adafruit128x64, I2C_ADDRESS);

  oled.setFont(System5x7);
  oled.clear();
  oled.print("Hello world!");
}
//------------------------------------------------------------------------------
void loop() {}