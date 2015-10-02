#ifndef XCOPTER_H
#define XCOPTER_H

#include "Motor.h"
#include "MPU6050.h"
#include "Vector.h"

bool QUAD_ARMED = false;

enum motor_pins {FRONT_LEFT = 2, FRONT_RIGHT = 3, BACK_LEFT = 5, BACK_RIGHT = 6};

Motor* motors[4];

MPU6050 accelgyro;

long timer;

float IMUPollPeriodMSec = 70;
float IMUPollPeriodSec = IMUPollPeriodMSec / 1000;

float input_adjustment = .25;
int armed_throttle = 5, armed_other = (int)(-50.0*input_adjustment+5.0);

float ACC_SENSITIVITY = 8192.0;
float GYRO_SENSITIVITY = 131.072;

int16_t ax, ay, az, ax_off, ay_off, az_off;
int16_t gx, gy, gz, gx_off, gy_off, gz_off;

float ax_acc, ay_acc, az_acc;
float gx_acc, gy_acc, gz_acc;

float p_roll = 1.5, d_roll, i_roll;
float p_pitch = 1.5, d_pitch, i_pitch;
float p_yaw, d_yaw, i_yaw;

// Period is 22 ms, duty cycle varies between 4 and 9 %
// MinRudd: 4.46 MaxRudd: 9.13
// MinElev: 4.46 MaxElev: 9.12
// MinAile: 4.46 MaxAile: 9.12
// MinThro: 4.48 MaxThro: 9.10
float ruddValue, elevValue, aileValue;
int ruddPin = A11, elevPin = A10, ailePin = A9, throPin = A8;
float controllerRuddValue, controllerElevValue, controllerAileValue, controllerThroValue; 

#endif