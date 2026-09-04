#include "IMU.h"
#include <Adafruit_MPU6050.h>

static Adafruit_MPU6050 mpu;
static sensors_event_t a, g, temp;
static float offset_deg = 5.3;
static float offset = offset_deg*PI/180.0;
static float gyroDrift = 0;

bool ImuBegin() {
    if (!mpu.begin()) return false;
    Wire.setClock(400000);
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    delay(100);
    gyroDrift = getGyroOffset();
    return true;
}

float get_accel_theta() {
    float xa = a.acceleration.x;
    float  za = a.acceleration.z;
    float theta = atan2(za, -1.0*xa) + offset;
    return theta;
};

float getGyroOffset() {
    float total_gy = 0;
    for (int i = 0; i < 100; i++) {
        mpu.getEvent(&a, &g, &temp);
        float gyro_y = -1.0*g.gyro.y;
        total_gy = total_gy + gyro_y;
        float gyro_y_deg = gyro_y*180.0/PI;
    }
    float average_gy = total_gy / 100.0;
    return average_gy;
};

ImuSample ImuRead() {
    mpu.getEvent(&a, &g, &temp);
    ImuSample s;
    s.time = micros();
    s.theta_dot = -1.0*g.gyro.y-gyroDrift;
    s.accel_theta = atan2(a.acceleration.z, -1.0*a.acceleration.x) + offset;
    return s;
};

