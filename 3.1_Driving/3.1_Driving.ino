/* This minimal example shows how to get single-shot range
measurements from the VL6180X.

The range readings are in units of mm. */

#include <Wire.h>
#include <VL6180X.h>
#include "DualEncoder.hpp"
#include "Motor.hpp"

VL6180X sensor;
int sensor_enable_pin = A1;

#define MOT1PWM 11  // PIN 9 is a PWM pin
#define MOT1DIR 12
#define MOT2PWM 9  // PIN 9 is a PWM pin
#define MOT2DIR 10
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);

#define set_distance 100
#define operation_speed 255
#define lidar_max_range 255

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  pinMode(sensor_enable_pin, OUTPUT);
  digitalWrite(sensor_enable_pin, HIGH);
  delay(50); //time for serial.print to start
  Serial.println("Initializing sensor...");
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);
  //sensor.setAddress(0x29);
}
void loop() 
{ 
  int distance = sensor.readRangeSingleMillimeters();
  Serial.print("distance:");
  Serial.println(distance);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();

  while (distance != set_distance) {
    if (distance > set_distance) {
      //go forward
      motor1.setPWM(output(distance));
      motor2.setPWM(-output(distance));
    } else {
      motor1.setPWM(-output(distance));
      motor2.setPWM(output(distance));
    }
    distance = sensor.readRangeSingleMillimeters();
    Serial.print("distance:");
    Serial.println(distance);
  }
}

int output(int distance) {
   int error = abs(distance - set_distance);
   float output = operation_speed * error/lidar_max_range;
   return static_cast<int>(output);
}
