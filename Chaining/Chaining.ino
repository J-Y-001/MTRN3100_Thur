#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"
#include "Wire.h"
#include <MPU6050_light.h>
#include "Motor.hpp"
#include "PIDController.hpp"
#include "path.h"

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
mtrn3100::EncoderOdometry encoder_odometry(15,87); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH 90
mtrn3100::IMUOdometry IMU_odometry;
mtrn3100::PIDController controller(350,1,0.5);
mtrn3100::PIDController controller2(350,1,0.5);
mtrn3100::PIDController IMUController(20,1,0.5);


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
}

int target_angle = 0;
int i = 0;
int check = 0;
int smooth_check = 0;
String command = path;

void loop() {
    delay(50);

    mpu.update();
    IMU_odometry.update(mpu.getAccX(), mpu.getAccY(), mpu.getAngleZ());
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

    if (command[i] == 'l' && check == 0) {
        target_angle = 90;
        IMUController.zeroAndSetTarget(mpu.getAngleZ(), target_angle);
        check = 1;
    }
    
    if (command[i] == 'r' && check == 0) {
        target_angle = -90;
        IMUController.zeroAndSetTarget(mpu.getAngleZ(), target_angle);
        check = 1;
    }

    if (command[i] == 'l' || command[i] == 'r'){
        float current_angle = mpu.getAngleZ();
        float pwm_L = IMUController.compute(current_angle);
        if (pwm_L > 100) {
            pwm_L = 100;
        }
        if (pwm_L < -100) {
            pwm_L = -100;
        }  
        motor.setPWM(-pwm_L);
        motor2.setPWM(-pwm_L);
    }

    if (command[i] == 'f' && check == 0) {
        controller.zeroAndSetTarget(encoder.getLeftRotation(), -11.25);
        controller2.zeroAndSetTarget(encoder.getRightRotation(), 11.25);
        check = 1;
    }

    if (command[i] == 'f') {
        float current_rotationL = encoder.getLeftRotation();
        float pwm_L = controller.compute(current_rotationL);
        float current_rotationR = encoder.getRightRotation();
        float pwm_R = controller2.compute(current_rotationR);  
        if (pwm_L > 100) {
            pwm_L = 100;
        }
        if (pwm_L < -100) {
            pwm_L = -100;
        }    
        if (pwm_R > 100) {
            pwm_R = 100;
        }
        if (pwm_R < -100) {
            pwm_R = -100;
        }     
        motor.setPWM(-pwm_L);
        motor2.setPWM(-pwm_R/1.05);
    }

    if (i + 2 < command.length()) {
        String three = command.substring(i, i + 3);

        if (three == 'frf' || three == 'flf') {
            if (smooth_check == 0) {
                controller.zeroAndSetTarget(encoder.getLeftRotation(), -11.25/2);
                controller2.zeroAndSetTarget(encoder.getRightRotation(), 11.25/2);
                smooth_check == 1;
            }

            if (smooth_check == 1) {
                float current_rotationL = encoder.getLeftRotation();
                float pwm_L = controller.compute(current_rotationL);
                float current_rotationR = encoder.getRightRotation();
                float pwm_R = controller2.compute(current_rotationR);  
                if (pwm_L > 100) {
                    pwm_L = 100;
                }
                if (pwm_L < -100) {
                    pwm_L = -100;
                }    
                if (pwm_R > 100) {
                    pwm_R = 100;
                }
                if (pwm_R < -100) {
                    pwm_R = -100;
                }     
                motor.setPWM(-pwm_L);
                motor2.setPWM(-pwm_R/1.05);

                if (encoder.getLeftRotation() == -11.25/2 && encoder.getRightRotation() == 11.25/2) {
                    smooth_check == 2;
                }
            }

            if (smooth_check == 2) {
                    if (three = 'flf') {
                        target_angle = 90;
                        IMUController.zeroAndSetTarget(mpu.getAngleZ(), target_angle);
                        check = 1;
                    }
                    
                    if ('frf') {
                        target_angle = -90;
                        IMUController.zeroAndSetTarget(mpu.getAngleZ(), target_angle);
                        check = 1;
                    }
                smooth_check == 3;
            }

            if (smooth_check == 3) {
                float current_angle = mpu.getAngleZ();
                if (three = 'flf') {
                    motor.setPWM(40);
                    motor2.setPWM(20);
                }
                if (three = 'frf') {
                    motor.setPWM(20);
                    motor2.setPWM(40);
                } 
                if (current_angle == target_angle) {
                    smooth_check == 4;
                }              
            }

            if (smooth_check == 4) {
                controller.zeroAndSetTarget(encoder.getLeftRotation(), -11.25/2);
                controller2.zeroAndSetTarget(encoder.getRightRotation(), 11.25/2);
                smooth_check == 5;
            }

            if (smooth_check == 5) {
                float current_rotationL = encoder.getLeftRotation();
                float pwm_L = controller.compute(current_rotationL);
                float current_rotationR = encoder.getRightRotation();
                float pwm_R = controller2.compute(current_rotationR);  
                if (pwm_L > 100) {
                    pwm_L = 100;
                }
                if (pwm_L < -100) {
                    pwm_L = -100;
                }    
                if (pwm_R > 100) {
                    pwm_R = 100;
                }
                if (pwm_R < -100) {
                    pwm_R = -100;
                }     
                motor.setPWM(-pwm_L);
                motor2.setPWM(-pwm_R/1.05);
                if (encoder.getLeftRotation() == -11.25/2 && encoder.getRightRotation() == 11.25/2) {
                    i = i + 2;
                }
            }
        }
    }

    //encoder_odometry.AMF();

    if (encoder_odometry.AMF() == 1) {
        i++;
        Serial.println(i);
        smooth_check = 0;
        check = 0;
        if (i == command.length()) {
           delay(20000);
        } 
    }
}
