#pragma once

#include "input.h"
#include "measurements.h"
#include "Motor.h"
#include "Arduino.h"

enum motor_pins {FRONT_LEFT = 2, FRONT_RIGHT = 3, BACK_LEFT = 5, BACK_RIGHT = 6};

extern Motor* motors[4];
extern int motor_power[4];

const float IMUPollPeriodMSec = 70;
const float IMUPollPeriodSec = IMUPollPeriodMSec / 1000;

void initializeMotors();
void controlFlight();
void writeMotors();
void setMotorValues();
void stopMotors();
void stabilize();
void rollPID();
void pitchPID();
void yawPID();

