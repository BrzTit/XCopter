#include "control.h"

Motor* motors[4];
int motor_power[4];

void initializeMotors()
{
	motors[0] = new Motor(FRONT_LEFT);
	motors[1] = new Motor(FRONT_RIGHT);
	motors[2] = new Motor(BACK_LEFT);
	motors[3] = new Motor(BACK_RIGHT);
}

void controlFlight()
{
	stabilize();
	setMotorValues();
}

void writeMotors()
{
	int i;
	for(i = 0; i < 4; i++)
	{
		motors[i]->write();
	}
}

void setMotorValues()
{
	// FL, FR, BL, BR
	motors[0]->setPower(motor_power[0]);
	motors[1]->setPower(motor_power[1]);
	motors[2]->setPower(motor_power[2]);
	motors[3]->setPower(motor_power[3]);
}


// Stops all motors.
void stopMotors()
{
	int i;
	for(i = 0; i < 4; i++)
	{
		motors[i]->stop();
	}
	writeMotors();
}

void stabilize()
{
	// motor_power = throttle +- roll PID +- pitch PID +- yawPID
	for(int i = 0; i < 4; i ++)
	{
		motor_power[i] = desired_values[0];
	}

	rollPID();
	pitchPID();
	yawPID();
}

void rollPID()
{
	float P = 10, I = 1, D = 0.1;
	static float error = 0, last_error = 0, total_error = 0;

	error = desired_values[1] - calculations[0];
	total_error += error;

	float power = P * error + I * total_error + D * (error - last_error) / IMUPollPeriodSec;

	// FL, FR, BL, BR
	motor_power[0] += -1 * power; 
	motor_power[1] += power;
	motor_power[2] += -1 * power;
	motor_power[3] += power;

	last_error = error;
}

void pitchPID()
{
	float P = 10, I = 1, D = 0.1;
	static float error = 0, last_error = 0, total_error = 0;

	error = desired_values[2] - calculations[1];
	total_error += error;

	float power = P * error + I * total_error + D * (error - last_error) / IMUPollPeriodSec;

	// FL, FR, BL, BR
	motor_power[0] += power; 
	motor_power[1] += power;
	motor_power[2] += -1 * power;
	motor_power[3] += -1 * power;

	last_error = error;

}

void yawPID()
{
// 	float P = 10, I = 1, D = 0.1;
// 	static float error = 0, last_error = 0, total_error = 0;
// 
// 	error = desired_values[1] - calculations[0];
// 	total_error += error;
// 
// 	float power = P * error + I * total_error + D * (error - last_error) / IMUPollPeriodSec;
// 
// 	// FL, FR, BL, BR
// 	motor_power[0] += 0; 
// 	motor_power[1] += 0;
// 	motor_power[2] += 0;
// 	motor_power[3] += 0;
// 
// 	last_error = error;

}
