
#include <Motors.h>
// Motor 1 pins
// Motor 1 pins

static const int freq = 5000;
static const int pwmChannel =0;
static const int resolution = 8;
static const int min_speed = 160;



void setPWM(Motor motor) {
    pinMode(motor.in1Pin, OUTPUT);
    pinMode(motor.in2Pin, OUTPUT);
    pinMode(motor.enablePin, OUTPUT);
    digitalWrite(motor.in1Pin, LOW);
    digitalWrite(motor.in2Pin, LOW);

    // configure LEDC PWM
    ledcAttach(motor.enablePin, freq, resolution); 
    ledcWrite(motor.enablePin, 0);
};

void run1Motor(Motor motor, int speed) {
    int speed_mag = abs(speed);
    if (speed_mag <= 255) {
        ledcWrite(motor.enablePin, speed_mag);
    } else {
        ledcWrite(motor.enablePin, 255);
    }
    if (speed > 0) {
        digitalWrite(motor.in1Pin, HIGH);
        digitalWrite(motor.in2Pin, LOW);
    } else {
        digitalWrite(motor.in1Pin, LOW);
        digitalWrite(motor.in2Pin, HIGH);
    }
}

void runBothMotors(Motor motor1, Motor motor2, int speed) {
    run1Motor(motor1, speed);
    run1Motor(motor2, speed);
}

void stopMotor(Motor motor) {
    ledcWrite(motor.enablePin, 0);
    digitalWrite(motor.in1Pin, 0);
    digitalWrite(motor.in2Pin, 0);
}

void stopBothMotors(Motor motor1, Motor motor2) {
    stopMotor(motor1);
    stopMotor(motor2);
}