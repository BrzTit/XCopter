#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor
{
public:
	Motor(int PIN);
	void write(); // Writes a speed to the motor; Input: 0 - 100
	void stop();
	void setPower(int p);
	

private:
	int power;
	int pin;
};

#endif
