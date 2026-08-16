#pragma once
#include <Arduino.h>

struct gyroData {
    float delta_theta;
    unsigned long time;
    float theta_dot;
};

struct ImuSample {
    unsigned long time;
    float accel_theta;
    float theta_dot;
};

float getGyroOffset();
float get_accel_theta();
bool ImuBegin();
ImuSample ImuRead();


