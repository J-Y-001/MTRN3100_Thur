#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"
#include "Wire.h"
#include <MPU6050_light.h>
#include "Motor.hpp"
#include "PIDController.hpp"

#define MOT1PWM 11
#define MOT1DIR 12
mtrn3100::Motor motor(MOT1PWM,MOT1DIR);

#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

MPU6050 mpu(Wire);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(15,99); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH 90
mtrn3100::IMUOdometry IMU_odometry;
mtrn3100::PIDController controller(8,0.2,0);
mtrn3100::PIDController controller2(8,0.2,0);



void setup() {
    Serial.begin(115200);
    Wire.begin();

    //Set up the IMU
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");
    controller.zeroAndSetTarget(encoder.getLeftRotation(), 0);
    controller2.zeroAndSetTarget(encoder.getRightRotation(), 0);
}


void loop() {

    delay(50);
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

    float current_rotation_L = encoder.getLeftRotation();
    float pwm_L = controller.compute(current_rotation_L);
    motor.setPWM(-pwm_L);

    float current_rotation_R = encoder.getRightRotation();
    float pwm_R = controller2.compute(current_rotation_R);
    motor2.setPWM(-pwm_R);

    for (int i = 0; i < command.length(); i++) {
        if (command[i] == 'l') {
            
        }
        if (command[i] == 'r') {

        }
        if (command[i] == 'f') {
            
        }
    }

}
