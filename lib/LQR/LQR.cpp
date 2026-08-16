#include "LQR.h"


static float K1 = 1.047;
static float K2 = .32;
static float u_sp = 1000;



int LQR_output(float theta, float theta_dot) {
    int output = (int)round(u_sp * (K1*theta + K2*theta_dot));
    return output;
};
