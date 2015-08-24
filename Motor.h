#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor
{
public:
	Motor(int PIN);
	void write(); // Writes a speed to the motor; Input: 0 - 100
	void stop();
	void setThrottle(int t);
	void setPitch(int p);
	void setYaw(int y);
	void setRoll(int r);
	void setConstant(int c);
	int getTotal();
	int getAdjustedTotal();

private:
	int pitch, roll, yaw, throttle, constant;
	int pin;
	

};

#endif
