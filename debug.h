#ifndef DEBUG_H
#define DEBUG_H

#include "control.h"
#include "input.h"
#include "measurements.h"
#include "Arduino.h"

void printMotorWriteValues();
void printControllerValues();
void printIMUValues();

#endif