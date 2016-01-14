#ifndef XCOPTER_H
#define XCOPTER_H

#include "Motor.h"
#include "MPU6050.h"
#include "Vector.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "Arduino.h"
#include "debug.h"
#include "control.h"
#include "measurements.h"
#include "input.h"

bool ARMED = false;

long timer;

float IMUPollPeriodMSec = 70;
float IMUPollPeriodSec = IMUPollPeriodMSec / 1000;

void setup();
void loop();
void checkArmed();
void checkDisarmed();

#endif