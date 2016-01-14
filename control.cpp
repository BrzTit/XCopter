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
	// DO SOME STUFF HERE WITH PID
	// stabilizeRoll();
	// stabilizePitch();
	// stabilizeYaw();
}
