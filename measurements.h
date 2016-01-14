#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include "Arduino.h"

MPU6050 accelgyro;

float ACC_SENSITIVITY = 16384.0;
float GYRO_SENSITIVITY = 131.072;

// Offset values
int16_t gx_off, gy_off, gz_off, ax_off, ay_off, az_off;

// Array containing the 6 measured values: ax, ay, az, gx, gy, gz
float measurements[6];

// Array containing the 2 calculated values: roll_angle, pitch_angle
float calculations[2];

void updateIMUValues();
float calcRollAngle();
float calcPitchAngle();
void initializeIMU();
void calibrateIMU();

#endif