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
    controller.zeroAndSetTarget(encoder.getLeftRotation(), -12.56);
    controller2.zeroAndSetTarget(encoder.getRightRotation(), 12.56);
}


void loop() {

//UNCOMMENT FOR TASK 2: 
//THE DELAY IS REQUIRED OTHERWISE THE ENCODER DIFFERENCE IS TOO SMALL
    delay(50);
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());


//UNCOMMET FOR TASK 3:
//NOTE: IMU ODOMETRY IS REALLY BAD, THIS TASK EXISTS TO TEACH YOU WHY IMU ODOMETRY SUCKS, DO NOT SPEND TOO LONG ON IT
    //mpu.update();
    //IMU_odometry.update(mpu.getAccX(),mpu.getAccY());
    float current_rotation_L = encoder.getLeftRotation();
    float pwm_L = controller.compute(current_rotation_L);
    motor.setPWM(-pwm_L);

    float current_rotation_R = encoder.getRightRotation();
    float pwm_R = controller2.compute(current_rotation_R);
    motor2.setPWM(-pwm_R);

    //Serial.print(encoder_odometry.AMF());
    int check = 0;

    if (encoder_odometry.AMF() == 1 && check == 0) {
        controller.zeroAndSetTarget(encoder.getLeftRotation(), 12.56);
        controller2.zeroAndSetTarget(encoder.getRightRotation(), -12.56);    
        delay(10000);
        //Serial.println(encoder.getLeftRotation());
        //Serial.println(encoder.getRightRotation());
        check = 1;   
    }
        Serial.print("Left: ");
        Serial.print(encoder.getLeftRotation());
        Serial.print(" Right: ");

        Serial.println(encoder.getRightRotation());

}
