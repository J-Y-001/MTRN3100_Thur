#pragma once

#include <Arduino.h>

namespace mtrn3100 {
class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), B(wheelBase), lastLPos(0), lastRPos(0), delta_left_radians(0), delta_right_radians(0), index(0), setup(0) {
        for (int i = 0; i < 10; i++) {
            arrayL[i] = 0;
            arrayR[i] = 0;
        }
    }

    //TODO: COMPLETE THIS FUNCTION
    void update(float leftValue,float rightValue) {
        leftValue = leftValue;
        //TODO: Calculate the change in radians since the last update.
        delta_left_radians = leftValue - lastLPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEYARE NOT THE WRONG DIRECTION 
        delta_right_radians = rightValue - lastRPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEY ARE NOT THE WRONG DIRECTION 
            
        float delta_s = R / 2 * delta_left_radians + R / 2 * delta_right_radians;
        float delta_h = -(R / B * delta_left_radians) + R / B * delta_right_radians;

        //TODO: Calculate the foward kinematics
        x += delta_s * cos(h);
        y += delta_s * sin(h);
        h += delta_h;
        lastLPos = leftValue;
        lastRPos = rightValue;
        //Serial.print("Left: ");
        //Serial.print(leftValue);
        //Serial.print(" Right: ");

        //Serial.println(rightValue);
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getH() const { return h; }

    float AMF() { //average moving filter
        arrayL[index] = delta_left_radians;
        arrayR[index] = delta_right_radians;

        float avgL = 0;
        float avgR = 0;

        for (int j = 0; j < 10; j++) {
            avgL += arrayL[j];
            avgR += arrayR[j];
        }

        avgL = avgL / 10;
        avgR = avgR / 10;

        index++;
        if (index == 10) {
            index = 0;
        }

        setup++;

        if (avgL == 0 && avgR == 0 && setup > 45) {
            setup = 0;
            return 1;
        }
        return 0;
    }

private:
    float x, y, h;
    const float R, B;
    float lastLPos, lastRPos;
    float delta_left_radians, delta_right_radians;
    float arrayL[10];
    float arrayR[10];
    int index;
    int setup;
};

}
