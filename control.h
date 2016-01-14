#pragma once

#include "input.h"
#include "measurements.h"
#include "Motor.h"
#include "Arduino.h"

enum motor_pins {FRONT_LEFT = 2, FRONT_RIGHT = 3, BACK_LEFT = 5, BACK_RIGHT = 6};

extern Motor* motors[4];
extern int motor_power[4];


// float p_roll = 1.5, d_roll, i_roll;
// float p_pitch = 1.5, d_pitch, i_pitch;
// float p_yaw, d_yaw, i_yaw;

void initializeMotors();
void controlFlight();
void writeMotors();
void setMotorValues();
void stopMotors();
void stabilize();
