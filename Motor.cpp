#include "Motor.h"
#include "Arduino.h"

Motor::Motor(int PIN)
{
	pitch = 0;
	roll = 0;
	yaw = 0;
	throttle = 0;
	constant = 0;

	pin = PIN;
	pinMode(pin, OUTPUT);
}

void Motor::setThrottle(int t) {throttle = t;}
void Motor::setPitch(int p) {pitch = p;}
void Motor::setYaw(int y) {yaw = y;}
void Motor::setRoll(int r) {roll = r;}
void Motor::setConstant(int c) {constant = c;}

int Motor::getTotal() {return throttle + yaw + roll + pitch;}

int Motor::getAdjustedTotal() {return throttle + yaw + roll + pitch - constant;}

void Motor::write()
{
	int total = getAdjustedTotal(); // total is between 0 and 100

	// 490 Hz = 2 ms period
	// 2ms pulse for full speed, 1 ms pulse for off
	int pwm_cycle = map(total, 0, 100, 128, 255);
	analogWrite(pin, pwm_cycle);
}

