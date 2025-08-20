#include <VL6180X.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

VL6180X sensor1;
VL6180X sensor2;
VL6180X sensor3;

int sensor1_pin = A1; // ENABLE PIN FOR SENSOR 1
int sensor2_pin = A0; // ENABLE PIN FOR SENSOR 2
int sensor3_pin = A2;

#define I2C_ADDRESS 0x3C
// Define proper RST_PIN if required.
#define RST_PIN -1
SSD1306AsciiWire oled;

void setup() { 
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);

  oled.setFont(System5x7);
  oled.clear();

  Serial.begin(9600);

  // SET UP ENABLE PINS AND DISABLE SENSORS
  pinMode(sensor1_pin, OUTPUT);
  pinMode(sensor2_pin, OUTPUT);
  pinMode(sensor3_pin, OUTPUT);
  digitalWrite(sensor1_pin, LOW);
  digitalWrite(sensor2_pin, LOW);
  digitalWrite(sensor3_pin, LOW);

  // ENABLE FIRST SENSOR AND CHANGE THE ADDRESS 
  digitalWrite(sensor1_pin, HIGH);
  delay(50);
  sensor1.init();
  sensor1.configureDefault();
  sensor1.setTimeout(250);
  sensor1.setAddress(0x30);
  delay(50);


  
  // ENABLE SECOND SENSOR AND CHANGE THE ADDRESS 
  // NOTE: WE DO NOT HAVE TO DISABLE THE FIRST SENSOR AS IT IS NOW ON A DIFFERENT ADDRESS 
  digitalWrite(sensor2_pin, HIGH);
  delay(50);
  sensor2.init(); //sets it default to 0x29
  sensor2.configureDefault();
  sensor2.setTimeout(250);
  sensor2.setAddress(0x31);

  digitalWrite(sensor3_pin, HIGH);
  delay(50);
  sensor3.init(); //sets it default to 0x29
  sensor3.configureDefault();
  sensor3.setTimeout(250);
  sensor3.setAddress(0x32);
}


void loop() {
  oled.clear();
  oled.print("Front:");
  oled.println(sensor1.readRangeSingleMillimeters());
  if (sensor1.timeoutOccurred()) { Serial.println("TIMEOUT  "); }


  oled.print("Left:");
  oled.println(sensor2.readRangeSingleMillimeters());
  if (sensor2.timeoutOccurred()) { Serial.println("  Sensor 2 TIMEOUT  "); }


  oled.print("Right:");
  oled.println(sensor3.readRangeSingleMillimeters());
  if (sensor3.timeoutOccurred()) { Serial.println("  Sensor 3 TIMEOUT  "); }

  delay(100);

}
