#include "Filter.h"

static float alpha = .98;
static float theta = 0;
static float gyroDriftTrack = 0;

void filterBegin(float theta0) {
    //Initiate value for static theta to be theta
    theta = theta0;
};

FiltData filterUpdate(float accelTheta, float thetaDot, float dt) {
    
    FiltData filtered;
    float deltaTheta = dt*thetaDot;
    // Update static gyroDriftTrack for this library
    gyroDriftTrack = gyroDriftTrack + deltaTheta;
    float gyro_filt_theta = theta + deltaTheta;
    // Update static theta in this library
    theta = alpha*gyro_filt_theta + (1-alpha)*accelTheta;
    // Write current values to filtered data struct
    filtered.accel_theta = accelTheta;
    filtered.gyro_drift_theta = gyroDriftTrack;
    filtered.theta = theta;
    filtered.theta_dot = thetaDot;
    return filtered;

}

