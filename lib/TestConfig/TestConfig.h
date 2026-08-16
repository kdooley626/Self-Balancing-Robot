#pragma once
#include <Arduino.h>

enum TestMode {IMU_only, motors_only, LQR_only, full_balance};
const TestMode ACTIVE_MODE = full_balance;