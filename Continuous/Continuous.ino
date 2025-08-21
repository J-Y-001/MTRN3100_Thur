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
int j = 0;
int check = 0;
int smooth_check = 0;
String command = path;

void loop() {
    delay(50);

    mpu.update();
    IMU_odometry.update(mpu.getAccX(), mpu.getAccY(), mpu.getAngleZ());
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

    if (j == 0 && check == 0) {
        target_angle = path[i][j];
        IMUController.zeroAndSetTarget(mpu.getAngleZ(), target_angle);
        check = 1;
    }

    if (j == 0){
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

    if (j == 1 && check == 0) {
        controller.zeroAndSetTarget(encoder.getLeftRotation(), (-11.25 * path[i][j]));
        controller2.zeroAndSetTarget(encoder.getRightRotation(), (11.25 * path[i][j]));
        check = 1;
    }

    if (j == 1) {
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

    //encoder_odometry.AMF();

    if (encoder_odometry.AMF() == 1) {
        if (j == 1) {
            i++;
            j = 0;
        }
        if (j == 0) {
            j = 1;
        }
        Serial.println(i);
        check = 0;
        if (i == command.length()) {
           delay(20000);
        } 
    }
}
