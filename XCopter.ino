#include "XCopter.h"


void setup() 
{
	Wire.begin();						// Initialize the I2C bus
	Serial.begin(38400);				// Begin the Serial Monitor

	initializeControllerInputPins();	// Initializes the pins for controller input
	initializeMotors();					// Creates 4 motor objects and stores them into motors[]

	// while(ARMED == false)				// Wait for quad to be armed
	// {
	// 	updateControllerValues();		// Poll and store controller values
	// 	checkArmed();					// Test whether controller is armed
	// }

	initializeIMU();					// Initialize and calibrate the IMU
	timer = millis();

}

void loop() 
{
	while(millis() - timer < IMUPollPeriodMSec)
	{
		// Do nothing
		// Makes sure parts of the program run at mostly constant time intervals
	}
	timer = millis();

	updateIMUValues();
	// updateControllerValues();

	printRawIMUValues();
	// printTranslatedIMUValues();
	// printFilteredIMUValues();
	// printControllerValues();
	// printMotorWriteValues();

	// controlFlight();

	// if(ARMED == true)
	// {
	// 	writeMotors();
	// 	checkDisarmed();
	// }
	// else
	// {
	// 	stopMotors();
	// 	checkArmed();
	// }
}

/*
Determines whether or not to arm the quadcopter. 
If the two sticks are pushed to the bottom left for 3 seconds, arms the quad.
Sets a global value indicating armed state. 
*/
void checkArmed()
{
	if((input[0] < 5) && (input[1] < -45) && (input[2] < -45) && (input[3] < -45))
	{
		ARMED = true;
		Serial.println("Armed.");
		delay(1000);
	}
}

/*
Determines whether or not to disarm the quadcopter. 
If the two sticks are pushed to the bottom left for 3 seconds, disarms the quad.
Sets a global value indicating armed state. 
*/
void checkDisarmed()
{
	if((input[0] < 5) && (input[1] < -45) && (input[2] < -45) && (input[3] < -45))
	{
		ARMED = false;
		Serial.println("Disarmed.");
		delay(1000);
	}	
}


