#pragma once

#include "control.h"
#include "input.h"
#include "measurements.h"
#include "Arduino.h"

void printMotorWriteValues();
void printControllerValues();
void printFilteredIMUValues();
void printRawIMUValues();
void printTranslatedIMUValues();
void printCalculatedAngles();
