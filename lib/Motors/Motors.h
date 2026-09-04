#pragma once
#include <Arduino.h>

struct Motor {
    int enablePin;
    int in1Pin;
    int in2Pin;
    int speed_min_pos;
    int speed_min_neg;
};

void setPWM(Motor motor);

void run1Motor(Motor motor, int speed);
void runBothMotors(Motor motor1, Motor motor2, int speed);
void stopMotor(Motor motor);
void stopBothMotors(Motor motor1, Motor motor2);