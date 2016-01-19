#pragma once

#include "MPU6050.h"
#include "Arduino.h"
#include "Math.h"
#include "control.h"

const float ACC_SENSITIVITY = 16384.0;
const float GYRO_SENSITIVITY = 131.072;
const int BUFFER_SIZE = 4;
const float GYRO_PERCENTAGE = 0.98;
const float ACC_PERCENTAGE = (1 - GYRO_PERCENTAGE);

// Array containing the 6 measured values: ax, ay, az, gx, gy, gz
extern float raw_measurements[6];
extern float translated_measurements[6];
extern float buffered_measurements[6][BUFFER_SIZE];
extern float filtered_measurements[6];

// Array containing the 2 calculated values: roll_angle, pitch_angle
extern float calculations[2];

void updateIMUValues();
void calcRollAngle();
void calcPitchAngle();
void initializeIMU();
bool calibrateIMU();
void filterMeasurements();
