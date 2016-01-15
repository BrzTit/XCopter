#include "input.h"

float input[4];
float desired_values[4];

void updateControllerValues()
{
	updateThrottle();
	updateRudder();
	updateElevation();
	updateAile();
	calculateDesiredValues();
}

void updateThrottle()
{
	float data = (pulseIn(THROTTLE_PIN, HIGH)*100.0)/22000.0;
	float raw_throttle = mapfloat(data, 4.4, 9.2, 0, 100);
	input[0] = raw_throttle;
}

void updateRudder()
{
	float data = (pulseIn(RUDDER_PIN, HIGH)*100.0)/22000.0;
	float raw_rudder = mapfloat(data, 4.4, 9.2, 0, 100) - 50.0;
	input[1] = raw_rudder;
}

void updateElevation()
{
	float data = (pulseIn(ELEVATION_PIN, HIGH)*100.0)/22000.0;
	float raw_elevation = mapfloat(data, 4.4, 9.2, 0, 100) - 50.0;
	input[2] = raw_elevation;
}

void updateAile()
{
	float data = (pulseIn(AILE_PIN, HIGH)*100.0)/22000.0;
	float raw_aile = mapfloat(data, 4.4, 9.2, 0, 100) - 50.0;
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

void calculateDesiredValues()
{
	// Array containing the four desired values: throttle, roll, pitch, yaw
	desired_values[0] = input[0];
	desired_values[1] = mapfloat(input[3],-50,50,45,-45);
	desired_values[2] = mapfloat(input[2],-50,50,45,-45);
	desired_values[3] = mapfloat(input[1],-50,50,45,-45);
}
