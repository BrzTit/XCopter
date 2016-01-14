#include "debug.h"

//Prints the combined value being written to each motor to the Serial Monitor.
void printMotorWriteValues()
{
	Serial.print(motors[0]->getAdjustedTotal()); Serial.print("\t");
	Serial.print(motors[1]->getAdjustedTotal()); Serial.print("\t");
	Serial.print(motors[2]->getAdjustedTotal()); Serial.print("\t");
	Serial.print(motors[3]->getAdjustedTotal()); Serial.println();
}

void printControllerValues()
{
	Serial.println("Throttle:  "); 	Serial.print(input[0]);
	Serial.println("Rudder:    "); 	Serial.print(input[1]); 
	Serial.println("Elevation: "); 	Serial.print(input[2]); 
	Serial.println("Aile:      "); 	Serial.print(input[3]); 
}

void printIMUValues()
{
	// display tab-separated accel/gyro x/y/z values
    Serial.print("ax: "); Serial.print(measurements[0]); Serial.print(" \t");
    Serial.print("ay: "); Serial.print(measurements[1]); Serial.print(" \t");
    Serial.print("az: "); Serial.print(measurements[2]); Serial.print(" \t");
    Serial.print("gx: "); Serial.print(measurements[3]); Serial.print(" \t");
    Serial.print("gy: "); Serial.print(measurements[4]); Serial.print("  \t");
    Serial.print("gz: "); Serial.println(measurements[5]);
}