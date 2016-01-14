#ifndef INPUT_H
#define INPUT_H

#include "Arduino.h"

// Period is 22 ms, duty cycle varies between 4 and 9 %
// MinRudd: 4.46 MaxRudd: 9.13
// MinElev: 4.46 MaxElev: 9.12
// MinAile: 4.46 MaxAile: 9.12
// MinThro: 4.48 MaxThro: 9.10

enum controller_pins {RUDDER_PIN = A11, ELEVATION_PIN = A10, AILE_PIN = A9, THROTTLE_PIN=A8}

// Array containing the four input values: throttle, rudder, elevation, aile
float input[4];

void updateControllerValues();
void updateThrottle();
void updateRudder();
void updateElevation();
void updateAile();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void initializeControllerInputPins();

#endif