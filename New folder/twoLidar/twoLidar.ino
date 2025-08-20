#include <VL6180X.h>
#include <Wire.h>

VL6180X sensor1;
VL6180X sensor2;
VL6180X sensor3;

int sensor1_pin = A1; // ENABLE PIN FOR SENSOR 1
int sensor2_pin = A0; // ENABLE PIN FOR SENSOR 2
int sensor3_pin = A2;

void setup() { 
  Wire.begin();
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
  Serial.print("Front:");
  Serial.print(sensor1.readRangeSingleMillimeters());
  if (sensor1.timeoutOccurred()) { Serial.print("TIMEOUT  "); }
  Serial.print(" | ");

  Serial.print("Left:");
  Serial.print(sensor2.readRangeSingleMillimeters());
  if (sensor2.timeoutOccurred()) { Serial.print("  Sensor 2 TIMEOUT  "); }
  Serial.print(" | ");

  Serial.print("Right:");
  Serial.print(sensor3.readRangeSingleMillimeters());
  if (sensor3.timeoutOccurred()) { Serial.print("  Sensor 3 TIMEOUT  "); }
  Serial.print(" | ");

  Serial.println();
  delay(100);

}
