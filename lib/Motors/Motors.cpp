
#include <Motors.h>
// Motor 1 pins
// Motor 1 pins

static const int freq = 5000;

static const int resolution = 8;

static const int to_zero = 10; // Motor inputs which should result in motors going to 0



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
    if (speed_mag <= to_zero) {
        speed = 0;
        ledcWrite(motor.enablePin, 0);
    } else {
        int comp_speed;
        if (speed <0 ) {
            comp_speed = speed_mag + motor.speed_min_neg;
        } else {
            comp_speed = speed_mag + motor.speed_min_pos;
        }
       
        if (comp_speed <= 255) {
            ledcWrite(motor.enablePin, comp_speed);
        } else {
            ledcWrite(motor.enablePin, 255);
        }
    }

    
    if (speed > 0) {
        digitalWrite(motor.in1Pin, HIGH);
        digitalWrite(motor.in2Pin, LOW);
    } else if (speed < 0) {
        digitalWrite(motor.in1Pin, LOW);
        digitalWrite(motor.in2Pin, HIGH);
    } else {
        digitalWrite(motor.in1Pin, LOW);
        digitalWrite(motor.in2Pin, LOW);
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