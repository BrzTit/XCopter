#include "debug.h"

//Prints the combined value being written to each motor to the Serial Monitor.
void printMotorWriteValues()
{
	Serial.print(motors[0]->getPower()); Serial.print("\t");
	Serial.print(motors[1]->getPower()); Serial.print("\t");
	Serial.print(motors[2]->getPower()); Serial.print("\t");
	Serial.print(motors[3]->getPower()); Serial.println();
}

void printControllerValues()
{
	Serial.println("Throttle:  "); 	Serial.print(input[0]);
	Serial.println("Rudder:    "); 	Serial.print(input[1]); 
	Serial.println("Elevation: "); 	Serial.print(input[2]); 
	Serial.println("Aile:      "); 	Serial.print(input[3]); 
}

void printRawIMUValues()
{
	// display tab-separated accel/gyro x/y/z values
    Serial.print("ax: "); Serial.print(raw_measurements[0]); Serial.print(" \t");
    Serial.print("ay: "); Serial.print(raw_measurements[1]); Serial.print(" \t");
    Serial.print("az: "); Serial.print(raw_measurements[2]); Serial.print(" \t");
    Serial.print("gx: "); Serial.print(raw_measurements[3]); Serial.print(" \t");
    Serial.print("gy: "); Serial.print(raw_measurements[4]); Serial.print("  \t");
    Serial.print("gz: "); Serial.println(raw_measurements[5]);
}

void printTranslatedIMUValues()
{
    // display tab-separated accel/gyro x/y/z values
    Serial.print("ax: "); Serial.print(translated_measurements[0]); Serial.print(" \t");
    Serial.print("ay: "); Serial.print(translated_measurements[1]); Serial.print(" \t");
    Serial.print("az: "); Serial.print(translated_measurements[2]); Serial.print(" \t");
    Serial.print("gx: "); Serial.print(translated_measurements[3]); Serial.print(" \t");
    Serial.print("gy: "); Serial.print(translated_measurements[4]); Serial.print("  \t");
    Serial.print("gz: "); Serial.println(translated_measurements[5]);
}

void printFilteredIMUValues()
{
	// display tab-separated accel/gyro x/y/z values
    Serial.print("ax: "); Serial.print(filtered_measurements[0]); Serial.print(" \t");
    Serial.print("ay: "); Serial.print(filtered_measurements[1]); Serial.print(" \t");
    Serial.print("az: "); Serial.print(filtered_measurements[2]); Serial.print(" \t");
    Serial.print("gx: "); Serial.print(filtered_measurements[3]); Serial.print(" \t");
    Serial.print("gy: "); Serial.print(filtered_measurements[4]); Serial.print("  \t");
    Serial.print("gz: "); Serial.println(filtered_measurements[5]);
}