#ifndef XCOPTER_H
#define XCOPTER_H

#include "Motor.h"
#include "MPU6050.h"
#include "Vector.h"

bool QUAD_ARMED = false;

enum motor_pins {FRONT_LEFT = 2, FRONT_RIGHT = 3, BACK_LEFT = 5, BACK_RIGHT = 6};
enum bank_direction {FRONT, BACK, LEFT, RIGHT};

Motor* motors[4];

MPU6050 accelgyro;

float input_adjustment = .25;
int armed_throttle = 5, armed_other = (int)(-50.0*input_adjustment+5.0);

float p_p = 10, p_r = 10, p_y = 10;
float i_p, i_r, i_y;
float d_p = 5, d_r = 5, d_y = 5; 

int16_t ax, ay, az, ax_off, ay_off, az_off;
int16_t gx, gy, gz, gx_off, gy_off, gz_off;

float ax_acc, ay_acc, az_acc;
float gx_acc, gy_acc, gz_acc;

Vector* hover_acc_vector = new Vector(0.0, 0.0, 1.0);
Vector* hover_gyro_vector = new Vector(0.0, 0.0, 0.0);
Vector* current_acc_vector;
Vector* current_gyro_vector;
Vector* previous_acc_vector;
Vector* previous_gyro_vector;




// Period is 22 ms, duty cycle varies between 4 and 9 %
// MinRudd: 4.46 MaxRudd: 9.13
// MinElev: 4.46 MaxElev: 9.12
// MinAile: 4.46 MaxAile: 9.12
// MinThro: 4.48 MaxThro: 9.10
float ruddValue, elevValue, aileValue, throValue;
int ruddPin = A11, elevPin = A10, ailePin = A9, throPin = A8; 

#endif