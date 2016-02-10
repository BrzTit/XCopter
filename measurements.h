#pragma once

#include "MPU6050.h"
#include "Arduino.h"
#include "Math.h"
#include "control.h"
#include <stlport.h>
#include <Eigen30.h>
#include <Eigen/LU>
#include <MS5611.h>
#include <Wire.h>


const float ACC_SENSITIVITY = 16384.0;
const float GYRO_SENSITIVITY = 131.072;
const int BUFFER_SIZE = 1;
const int NUM_MEASUREMENTS = 7;

// Array containing the 6 measured values: ax, ay, az, gx, gy, gz
extern float raw_measurements[NUM_MEASUREMENTS];
extern float translated_measurements[NUM_MEASUREMENTS];
extern float buffered_measurements[NUM_MEASUREMENTS][BUFFER_SIZE];
extern float filtered_measurements[NUM_MEASUREMENTS];

// Array containing the 2 calculated values: roll_angle, pitch_angle
extern float calculations[2];


void calcRollAngle();
void calcPitchAngle();
void calcYawAngle();

void filterMeasurements();
void initKalmanValues();

void initializeSensors();
void initializeIMU();
void initializeBarometer();

void updateSensors();
void updateIMUValues();
void updateBarometerValues();

bool calibrateIMU();
