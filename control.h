#ifndef CONTROL_H
#define CONTROL_H

#include "input.h"
#include "measurements.h"
#include "Arduino.h"

Motor* motors[4];

enum motor_pins {FRONT_LEFT = 2, FRONT_RIGHT = 3, BACK_LEFT = 5, BACK_RIGHT = 6};

int motor_power[4];


float p_roll = 1.5, d_roll, i_roll;
float p_pitch = 1.5, d_pitch, i_pitch;
float p_yaw, d_yaw, i_yaw;

void initializeMotors();
void controlFlight();
void writeMotors();
void setMotorValues();
void stopMotors();
void stabilize();

#endif