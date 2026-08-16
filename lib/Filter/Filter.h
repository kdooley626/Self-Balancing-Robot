#pragma once
#include <Arduino.h>

struct FiltData {
    float theta;
    float gyro_drift_theta;
    float accel_theta;
    float theta_dot;
};

void filterBegin(float theta0);
FiltData filterUpdate(float accelTheta, float thetaDot, float dt);