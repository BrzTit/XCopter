#pragma once

#include "Arduino.h"

class Motor
{
public:
	Motor(int PIN);
	void write(); // Writes a speed to the motor; Input: 0 - 100
	void stop();
	void setPower(int p);
	int getPower();
	

private:
	int power;
	int pin;
};

