#pragma once

#include "MPU6050.h"
#include "Arduino.h"

const float ACC_SENSITIVITY = 16384.0;
const float GYRO_SENSITIVITY = 131.072;

// Array containing the 6 measured values: ax, ay, az, gx, gy, gz
extern float measurements[6];

// Array containing the 2 calculated values: roll_angle, pitch_angle
extern float calculations[2];

void updateIMUValues();
float calcRollAngle();
float calcPitchAngle();
void initializeIMU();
void calibrateIMU();
