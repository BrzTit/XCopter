#include "input.h"

void updateControllerValues()
{
	updateThrottle();
	updateRudder();
	updateElevation();
	updateAile();
}

void updateThrottle()
{
	float data = (pulseIn(THROTTLE_PIN, HIGH)*100.0)/22000.0;
	raw_throttle = mapfloat(data, 4.4, 9.2, 0, 100);
	input[0] = raw_throttle;
}

void updateRudder()
{
	float data = (pulseIn(RUDDER_PIN, HIGH)*100.0)/22000.0;
	raw_rudder = mapfloat(data, 4.4, 9.2, 0, 100) - 50.0;
	input[1] = raw_rudder;
}

void updateElevation()
{
	float data = (pulseIn(ELEVATION_PIN, HIGH)*100.0)/22000.0;
	raw_elevation = mapfloat(data, 4.4, 9.2, 0, 100) - 50.0;
	input[2] = raw_elevation;
}

void updateAile()
{
	float data = (pulseIn(AILE_PIN, HIGH)*100.0)/22000.0;
	raw_aile = mapfloat(data, 4.4, 9.2, 0, 100) - 50.0;
	input[3] = raw_aile;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void initializeControllerInputPins()
{
	pinMode(RUDDER_PIN, INPUT);
	pinMode(ELEVATION_PIN, INPUT);
	pinMode(AILE_PIN, INPUT);
	pinMode(THROTTLE_PIN, INPUT);
}
