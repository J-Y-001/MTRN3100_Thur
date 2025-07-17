#ifndef IMU_ODOMETRY_HPP
#define IMU_ODOMETRY_HPP

#include <Arduino.h>

namespace mtrn3100 {
    class IMUOdometry {
    public:
        IMUOdometry() : x(0), y(0), vx(0), vy(0), lastUpdateTime(millis()), angleZ(0) {}

        void update(float accel_x, float accel_y, float angle_z) {
            unsigned long currentTime = millis();
            float dt = (currentTime - lastUpdateTime);  // Convert to seconds
            lastUpdateTime = currentTime;

            // Integrate acceleration to get velocity
            vx += accel_x * dt/1000;
            vy += accel_y * dt/1000;

            // TODO: Integrate velocity to get position
            x += vx * dt/1000;
            y += vy * dt/1000;

            angleZ = angle_z;
        }

        float getX() const { return x; }
        float getY() const { return y; }
        float getAngleZ() const { return angleZ; }

    private:
        float x, y;
        float vx, vy;
        unsigned long lastUpdateTime;
        float angleZ;
    };
}

#endif // IMU_ODOMETRY_HPP
