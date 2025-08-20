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
int skip = 0;
int smooth_end = 0;
int left_target = 0;
int right_target = 0;
int global_angle_target = 0;
String three = "";

void loop() {
    delay(50);

    mpu.update();
    IMU_odometry.update(mpu.getAccX(), mpu.getAccY(), mpu.getAngleZ());
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

    if (i + 3 <= command.length() ) {
        three = command.substring(i, i + 3);
    }

    if (three == "FRF" || three == "FLF") {
        Serial.println(smooth_check);
        if (three == "FRF" || three == "FLF") {
            if (smooth_check == 0) {
                controller.zeroAndSetTarget(encoder.getLeftRotation(), -11.25/2);
                controller2.zeroAndSetTarget(encoder.getRightRotation(), 11.25/2);
                left_target = encoder.getLeftRotation() - 11.25/2;
                right_target = encoder.getRightRotation() + 11.25/2;
                smooth_check = 1;
                Serial.println(smooth_check);
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

                if (current_rotationL <= left_target && current_rotationR >= right_target) {
                    smooth_check = 2;
                    Serial.println(smooth_check);
                }
            }

            if (smooth_check == 2) {
                    if (three == "FLF") {
                        target_angle = 90;
                        IMUController.zeroAndSetTarget(mpu.getAngleZ(), target_angle);
                        global_angle_target = mpu.getAngleZ() + target_angle;
                    }
                    
                    if (three == "FRF") {
                        target_angle = -90;
                        IMUController.zeroAndSetTarget(mpu.getAngleZ(), target_angle);
                        global_angle_target = mpu.getAngleZ() + target_angle;
                    }
                smooth_check = 3;
                Serial.println(smooth_check);
            }

            if (smooth_check == 3) {
                float current_angle = mpu.getAngleZ();
                float pwm_L = IMUController.compute(current_angle);
                if (three == "FLF") {
                    if (pwm_L > 100) {
                        pwm_L = 100;
                    }
                    if (pwm_L < -100) {
                        pwm_L = -100;
                    }  
                    motor.setPWM(pwm_L/2);
                    motor2.setPWM(-pwm_L);
                }
                if (three == "FRF") {
                    if (pwm_L > 100) {
                        pwm_L = 100;
                    }
                    if (pwm_L < -100) {
                        pwm_L = -100;
                    }  
                    motor.setPWM(-pwm_L);
                    motor2.setPWM(pwm_L/2);
                } 
                if (three == "FLF" && abs(global_angle_target - mpu.getAngleZ()) < 2) {
                    smooth_check = 4;
                    Serial.println(smooth_check);
                }   
                if (three == "FRF" && abs(global_angle_target - mpu.getAngleZ()) < 2) {
                    smooth_check = 4;
                    Serial.println(smooth_check);
                }             
            }

            if (smooth_check == 4) {
                controller.zeroAndSetTarget(encoder.getLeftRotation(), -11.25/2);
                controller2.zeroAndSetTarget(encoder.getRightRotation(), 11.25/2);
                left_target = encoder.getLeftRotation() - 11.25/2;
                right_target = encoder.getRightRotation() + 11.25/2;
                smooth_check = 5;
                Serial.println(smooth_check);
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
                if (current_rotationL <= left_target && current_rotationR >= right_target && smooth_end == 0) {
                    skip = 1;
                    motor.setPWM(0);
                    motor2.setPWM(0);
                    if (i + 3 <= command.length()) {
                        i = i + 3;
                    }
                    else if (i + 3 == command.length()) {
                        delay(20000);
                    }
                    smooth_end = 1;
                    Serial.println("END");
                    Serial.println(i);
                    three = "";
                }
            }
        }
    }

    else {
        if (command[i] == 'L' && check == 0 && smooth_check == 0) {
            target_angle = 90;
            IMUController.zeroAndSetTarget(mpu.getAngleZ(), target_angle);
            global_angle_target = mpu.getAngleZ() + target_angle;
            check = 1;
        }
        
        if (command[i] == 'R' && check == 0 && smooth_check == 0) {
            target_angle = -90;
            IMUController.zeroAndSetTarget(mpu.getAngleZ(), target_angle);
            global_angle_target = mpu.getAngleZ() + target_angle;
            check = 1;
            Serial.println("RIGHT TURN");
        }

        if (command[i] == 'L' || command[i] == 'R'){
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
            Serial.println(i);
            if (abs(current_angle - global_angle_target) < 2) {
                motor.setPWM(0);
                motor2.setPWM(0);
            }
        }

        if (command[i] == 'F' && check == 0 && smooth_check == 0) {
            controller.zeroAndSetTarget(encoder.getLeftRotation(), -11.25);
            controller2.zeroAndSetTarget(encoder.getRightRotation(), 11.25);
            left_target = encoder.getLeftRotation() - 11.25;
            right_target = encoder.getRightRotation() + 11.25;
            Serial.println("DRIVE FORWARD");
            check = 1;
        }

        if (command[i] == 'F' && smooth_check == 0) {
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
    }
    //encoder_odometry.AMF();

    if (encoder_odometry.AMF() == 1) {
        if (skip == 1) {
            skip = 0;
            smooth_check = 0;
            smooth_end = 0;
            controller.zeroAndSetTarget(encoder.getLeftRotation(), encoder.getLeftRotation());
            controller2.zeroAndSetTarget(encoder.getRightRotation(), encoder.getRightRotation());
            Serial.println("SKIP");
        }
        else {
            i++;
        }
        check = 0;
        if (i == command.length()) {
           delay(20000);
        } 
    }
}
