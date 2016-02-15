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
#include <HMC5883L.h>


const float ACC_SENSITIVITY = 16384.0;
const float GYRO_SENSITIVITY = 131.072;
const int BUFFER_SIZE = 1;
const int NUM_MEASUREMENTS = 7;

// Array containing the measured values: ax, ay, az, gx, gy, gz, barometer
extern float raw_measurements[NUM_MEASUREMENTS];
extern float translated_measurements[NUM_MEASUREMENTS];
extern float buffered_measurements[NUM_MEASUREMENTS][BUFFER_SIZE];
extern float filtered_measurements[NUM_MEASUREMENTS];

// Array containing the 2 calculated values: roll_angle, pitch_angle
extern float calculations[3];


void calcRollAngle();
void calcPitchAngle();
void calcYawAngle();

void filterMeasurements();
void initKalmanValues();

void initializeSensors();
void initializeIMU();
void initializeBarometer();
void initializeCompass();

void updateSensors();
void updateIMUValues();
void updateBarometerValues();
void updateCompassValues();

bool calibrateIMU();
