#include "XCopter.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "Arduino.h"


void setup() {

	initializeWire();					// Initialize the I2C bus
	initializeControllerInputPins();	// Initializes the pins for controller input
	initializeMotors();					// Creates 4 motor objects and stores them into motors[]

	Serial.begin(38400);				// Begin the Serial Monitor

	while(QUAD_ARMED == false)				// Wait for quad to be armed
	{
		updateControllerValues();		// Poll and store controller values
		checkArmed();					// Test whether controller is armed
	}

	initializeIMU();					// Initialize and calibrate the IMU
	timer = millis();

}

void loop() {

	while(millis() - timer < IMUPollPeriodMSec)
	{
		// Do nothing
	}
	timer = millis();

	updateIMUValues();
	//printIMUValues();
	//printVectorValues();

	updateControllerValues();
	//printControllerValues();
	//printMotorWriteValues();

	controlFlight();

	if(QUAD_ARMED)
	{
		writeMotors();
		checkDisarmed();
	}
	else
	{
		stopMotors();
		checkArmed();
	}


}

/*
Stops all motors.
Sets all of the motor variables to 0, then writes them.
*/
void stopMotors()
{
	int i;
	for(i = 0; i < 4; i++)
	{
		motors[i]->stop();
	}
	writeMotors();
}

/*
Prints the combined value being written to each motor to the Serial Monitor.
*/
void printMotorWriteValues()
{
	Serial.print(motors[0]->getAdjustedTotal()); Serial.print("\t");
	Serial.print(motors[1]->getAdjustedTotal()); Serial.print("\t");
	Serial.print(motors[2]->getAdjustedTotal()); Serial.print("\t");
	Serial.print(motors[3]->getAdjustedTotal()); Serial.println();
}

/*
Determines whether or not to arm the quadcopter. 
If the two sticks are pushed to the bottom left, arms the quad.
Sets a global value indicating armed state. 
Delays 1 second in order to prevent disarming.
*/
void checkArmed()
{
	if( (controllerThroValue < armed_throttle) && (controllerRuddValue < armed_other) && (controllerElevValue < armed_other) && (controllerAileValue < armed_other))
	{
		Serial.println(armed_other);
		QUAD_ARMED = true;
		Serial.println("Armed.");
		delay(1000);
	}
}

/*
Determines whether or not to disarm the quadcopter. 
If the two sticks are pushed to the bottom left, disarms the quad.
Sets a global value indicating armed state. 
Delays 1 second in order to prevent rearming.
*/
void checkDisarmed()
{
	if( (controllerThroValue < armed_throttle) && (controllerRuddValue < armed_other) && (controllerElevValue < armed_other) && (controllerAileValue < armed_other))
	{
		QUAD_ARMED = false;
		Serial.println("Disarmed.");
		delay(1000);
	}	
}

void printVectorValues()
{
	/*
	Serial.print("a/g:\t");
    Serial.print("x: "); Serial.print(current_acc_vector->x); Serial.print(" \t");
    Serial.print("y: "); Serial.print(current_acc_vector->y); Serial.print(" \t");
    Serial.print("z: "); Serial.print(current_acc_vector->z); Serial.print(" \t");
    Serial.print("x: "); Serial.print(current_gyro_vector->x); Serial.print(" \t");
    Serial.print("y: "); Serial.print(current_gyro_vector->y); Serial.print("  \t");
    Serial.print("z: "); Serial.println(current_gyro_vector->z);
    */
}

void writeMotors()
{
	int i;
	for(i = 0; i < 4; i++)
	{
		motors[i]->write();
	}
}

void initializeControllerInputPins()
{
	pinMode(ruddPin, INPUT);
	pinMode(elevPin, INPUT);
	pinMode(ailePin, INPUT);
	pinMode(throPin, INPUT);
}

void printControllerValues()
{
	Serial.print("Throttle: "); Serial.print(controllerThroValue); Serial.print("\t");
	Serial.print("Rudder: "); Serial.print(ruddValue); Serial.println();
	Serial.print("Aile: "); Serial.print(aileValue); Serial.print("\t");
	Serial.print("Elev: "); Serial.print(elevValue); Serial.println(); Serial.println();
}

void updateControllerValues()
{
	updateThrottle();
	updateRudder();
	updateElev();
	updateAile();
}

void controlFlight()
{
	stabilize();

	setMotorValues();
}

void setMotorValues()
{
	// FL, FR, BL, BR
	motors[0]->setThrottle(controllerThroValue);
	motors[1]->setThrottle(controllerThroValue);
	motors[2]->setThrottle(controllerThroValue);
	motors[3]->setThrottle(controllerThroValue);

	motors[0]->setYaw(ruddValue);
	motors[1]->setYaw(-ruddValue);
	motors[2]->setYaw(-ruddValue);
	motors[3]->setYaw(ruddValue);

	motors[0]->setPitch(-elevValue);
	motors[1]->setPitch(-elevValue);
	motors[2]->setPitch(elevValue);
	motors[3]->setPitch(elevValue);

	motors[0]->setRoll(aileValue);
	motors[1]->setRoll(-aileValue);
	motors[2]->setRoll(aileValue);
	motors[3]->setRoll(-aileValue);

	int i;
	int constant_offset = 0;
	for(i=0; i<4; i++)
	{
		 int total = motors[i]->getTotal();
		 if(total >= 100 && (total - 100) > constant_offset)
		 {
		 	constant_offset = total - 100;
		 }
	}
	
	for(i=0;i<4;i++)
	{
		motors[i]->setConstant(constant_offset);
		if(motors[i]->getAdjustedTotal() < 0)
		{
			motors[i]->setConstant(constant_offset+motors[i]->getAdjustedTotal());
		}
	}
	
}

/*

*/
void stabilize()
{
	stabilizeRoll();
	stabilizePitch();
	//stabilizeYaw();
}

/*

*/
void updateThrottle()
{
	float inputThroValue = (pulseIn(throPin, HIGH)*100.0)/22000.0;
	controllerThroValue = mapfloat(inputThroValue, 4.4, 9.2, 0, 100);
}

/*

*/
void updateRudder()
{
	float inputRuddValue = (pulseIn(ruddPin, HIGH)*100.0)/22000.0;
	controllerRuddValue = input_adjustment*(mapfloat(inputRuddValue, 4.4, 9.2, 0, 100) - 50.0);
}

/*

*/
void updateElev()
{
	float inputElevValue = (pulseIn(elevPin, HIGH)*100.0)/22000.0;
	controllerElevValue = input_adjustment*(mapfloat(inputElevValue, 4.4, 9.2, 0, 100) - 50.0);
}

/*

*/
void updateAile()
{
	float inputAileValue = (pulseIn(ailePin, HIGH)*100.0)/22000.0;
	controllerAileValue = input_adjustment*(mapfloat(inputAileValue, 4.4, 9.2, 0, 100) - 50.0);
}

/*

*/
void initializeMotors()
{
	motors[0] = new Motor(FRONT_LEFT);
	motors[1] = new Motor(FRONT_RIGHT);
	motors[2] = new Motor(BACK_LEFT);
	motors[3] = new Motor(BACK_RIGHT);
}

// Initializes the I2C bus
void initializeWire()
{
    Wire.begin();
}

/*

*/
void printIMUValues()
{
	// display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print("x: "); Serial.print(ax_acc); Serial.print(" \t");
    Serial.print("y: "); Serial.print(ay_acc); Serial.print(" \t");
    Serial.print("z: "); Serial.print(az_acc); Serial.print(" \t");
    Serial.print("x: "); Serial.print(gx_acc); Serial.print(" \t");
    Serial.print("y: "); Serial.print(gy_acc); Serial.print("  \t");
    Serial.print("z: "); Serial.println(gz_acc);
}

/*

*/
void updateIMUValues()
{
	// read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Account for offset
    ax += -1*ax_off;
    ay += -1*ay_off;
    az += -1*az_off;

    gx += -1*gx_off;
    gy += -1*gy_off;
    gz += -1*gz_off;

    // Translate into terms of g.
    ax_acc = (float)ax/ACC_SENSITIVITY;
    ay_acc = (float)ay/ACC_SENSITIVITY;
    az_acc = (float)az/ACC_SENSITIVITY;

    // Translate into terms of degrees/s
    gx_acc = (float)gx/GYRO_SENSITIVITY;
    gy_acc = (float)gy/GYRO_SENSITIVITY;
    gz_acc = (float)gz/GYRO_SENSITIVITY;

}

/*

*/
void initializeIMU()
{
	Serial.println("Setting IMU offsets to 0...");
	accelgyro.setXAccelOffset(0); 
    accelgyro.setYAccelOffset(0); 
    accelgyro.setZAccelOffset(0); 

    accelgyro.setXGyroOffset(0); 
    accelgyro.setYGyroOffset(0); 
    accelgyro.setZGyroOffset(0); 

	// initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Serial.println("Calibrating the IMU...");
    calibrateIMU();
    Serial.println("Finished calibration.");
    
}

// Takes an average of 1000 readings from the IMU and averages them to get the offset.
void calibrateIMU()
{
	float totalAX = 0, totalAY = 0, totalAZ = 0;
	float totalGX = 0, totalGY = 0, totalGZ = 0;

	Serial.println("Totalling IMU Values...");
	for(int i = 0; i<1000;i++)
	{
		updateIMUValues();

		totalAX += ax;
		totalAY += ay;
		totalAZ += az;

		totalGX += gx;
		totalGY += gy;
		totalGZ += gz;
	}
	Serial.println("Finished totaling...");

	Serial.println("Averaging Total Values...");
	ax_off = (int)(totalAX/1000.0);
	ay_off = (int)(totalAY/1000.0);
	az_off = (int)(totalAZ/1000.0) - ACC_SENSITIVITY;

	gx_off = (int)(totalGX/1000.0);
	gy_off = (int)(totalGY/1000.0);
	gz_off = (int)(totalGZ/1000.0);
}

/*

*/
void stabilizeRoll()
{
	float currentRollAngle = calcRollAngle();
	float desiredRollAngle = mapfloat(controllerAileValue, -50, 50, -45, 45);

	float difference = currentRollAngle - desiredRollAngle;

	aileValue = p_roll * difference;
}

float calcRollAngle()
{
	static float rollAngle;
	// angle = 0.98 * (angle + gyrData * dt) + 0.02 * (accData)
	// Roll along y-axis, pitch along x-axis
	// Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -4 to 4 G at 16Bit -> 2G = 16384 && 0.5G = 4096

    float rollAcc = 0.0;
	int forceMagnitudeApprox = abs(ax) + abs(ay) + abs(az);
	if (forceMagnitudeApprox < 16384 && forceMagnitudeApprox > 4096)
    {
		// Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f(ax_acc, az_acc) * 180 / 3.14159265359;
    }
    else
    {
    	printIMUValues();
    }

	rollAngle = 0.98 * (rollAngle + -1*gy_acc * IMUPollPeriodSec) + 0.02 * (rollAcc);
	// Serial.print("Roll Angle: ");
	// Serial.println(rollAngle);

	return rollAngle;
}

/*

*/
void stabilizePitch()
{
	float currentPitchAngle = calcPitchAngle();
	float desiredPitchAngle = mapfloat(controllerElevValue, -50, 50, -45, 45);

	float difference = currentPitchAngle - desiredPitchAngle;

	elevValue = p_pitch * difference;
}

float calcPitchAngle()
{
	static float pitchAngle;
	// angle = 0.98 * (angle + gyrData * dt) + 0.02 * (accData)
	// Roll along y-axis, pitch along x-axis
	// Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -4 to 4 G at 16Bit -> 2G = 16384 && 0.5G = 4096

    float pitchAcc = 0.0;
	int forceMagnitudeApprox = abs(ax) + abs(ay) + abs(az);
	if (forceMagnitudeApprox < 16384 && forceMagnitudeApprox > 4096)
    {
		// Turning around the Y axis results in a vector on the X-axis
        pitchAcc = atan2f(ay_acc, az_acc) * 180 / 3.14159265359;
    }
    else
    {
    	printIMUValues();
    }

	pitchAngle = 0.98 * (pitchAngle + gx_acc * IMUPollPeriodSec) + 0.02 * (pitchAcc);
	//Serial.print("Pitch Angle: ");
	//Serial.println(pitchAngle);

	return pitchAngle;
}

/*

*/
void stabilizeYaw()
{

}

/*

*/
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

