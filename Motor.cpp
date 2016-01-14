#include "Motor.h"

Motor::Motor(int PIN)
{
	power = 0;

	pin = PIN;
	pinMode(pin, OUTPUT);
}

void Motor::write()
{
	// 490 Hz = 2 ms period
	// 2ms pulse for full speed, 1 ms pulse for off
	int pwm_cycle = map(power, 0, 100, 128, 255);
	analogWrite(pin, pwm_cycle);
}

void Motor::stop()
{
	power = 0;
}

void Motor::setPower(int p)
{
	power = p
}

