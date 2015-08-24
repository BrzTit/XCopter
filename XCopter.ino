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

}

void loop() {

	updateIMUValues();
	//printIMUValues();
	//printVectorValues();

	updateControllerValues();
	//printControllerValues();
	//printMotorWriteValues();

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
	if( (throValue < armed_throttle) && (ruddValue < armed_other) && (elevValue < armed_other) && (aileValue < armed_other))
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
	if( (throValue < armed_throttle) && (ruddValue < armed_other) && (elevValue < armed_other) && (aileValue < armed_other))
	{
		QUAD_ARMED = false;
		Serial.println("Disarmed.");
		delay(1000);
	}	
}

void printVectorValues()
{
	Serial.print("a/g:\t");
    Serial.print("x: "); Serial.print(current_acc_vector->x); Serial.print(" \t");
    Serial.print("y: "); Serial.print(current_acc_vector->y); Serial.print(" \t");
    Serial.print("z: "); Serial.print(current_acc_vector->z); Serial.print(" \t");
    Serial.print("x: "); Serial.print(current_gyro_vector->x); Serial.print(" \t");
    Serial.print("y: "); Serial.print(current_gyro_vector->y); Serial.print("  \t");
    Serial.print("z: "); Serial.println(current_gyro_vector->z);
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
	Serial.print("Throttle: "); Serial.print(throValue); Serial.print("\t");
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

	int controller_aile = aileValue;
	int controller_elev = elevValue;

	// If not touching controls, stabilize
	if(abs(controller_aile) < 2)
	{
		stabilizeRoll();
	}

	if(abs(controller_elev) < 2)
	{
		stabilizePitch();
	}
	//stabilizeYaw();

	// FL, FR, BL, BR
	motors[0]->setThrottle(throValue);
	motors[1]->setThrottle(throValue);
	motors[2]->setThrottle(throValue);
	motors[3]->setThrottle(throValue);

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
		 if(total >= 100 && total > constant_offset)
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

void updateThrottle()
{
	throValue = (pulseIn(throPin, HIGH)*100.0)/22000.0;
	throValue = mapfloat(throValue, 4.4, 9.2, 0, 100);
}

void updateRudder()
{
	ruddValue = (pulseIn(ruddPin, HIGH)*100.0)/22000.0;
	ruddValue = input_adjustment*(mapfloat(ruddValue, 4.4, 9.2, 0, 100) - 50.0);
}

void updateElev()
{
	elevValue= (pulseIn(elevPin, HIGH)*100.0)/22000.0;
	elevValue = input_adjustment*(mapfloat(elevValue, 4.4, 9.2, 0, 100) - 50.0);
}

void updateAile()
{
	aileValue = (pulseIn(ailePin, HIGH)*100.0)/22000.0;
	aileValue = input_adjustment*(mapfloat(aileValue, 4.4, 9.2, 0, 100) - 50.0);
}

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
    ax_acc = (float)ax/4096.0;
    ay_acc = (float)ay/4096.0;
    az_acc = (float)az/4096.0;

    // Translate into terms of degrees/s
    gx_acc = (float)gx/65.5;
    gy_acc = (float)gy/65.5;
    gz_acc = (float)gz/65.5;

    // Update the vectors
    delete(previous_acc_vector);
    delete(previous_gyro_vector);
    previous_acc_vector=new Vector(current_acc_vector->x, current_acc_vector->y, current_acc_vector->z);
    previous_gyro_vector=new Vector(current_gyro_vector->x, current_gyro_vector->y, current_gyro_vector->z);

    delete(current_acc_vector);
    delete(current_gyro_vector);
    current_acc_vector=new Vector(ax_acc, ay_acc, az_acc);
    current_gyro_vector=new Vector(gx_acc, gy_acc, gz_acc);
}

void initializeIMU()
{
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

	ax_off = (int)(totalAX/1000.0);
	ay_off = (int)(totalAY/1000.0);
	az_off = (int)(totalAZ/1000.0) - 4096;

	gx_off = (int)(totalGX/1000.0);
	gy_off = (int)(totalGY/1000.0);
	gz_off = (int)(totalGZ/1000.0);
}

void stabilizeRoll()
{
	float p_ax = (hover_acc_vector->x - current_acc_vector->x);

	//						Current error					-				previous error
	//float d_ax = (hover_acc_vector->x - current_acc_vector->x) - (hover_acc_vector->x - previous_acc_vector->x);

	aileValue += (-p_r * p_ax);// + (-d_r * d_ax);
}

void stabilizePitch()
{
	float p_ay = (hover_acc_vector->y - current_acc_vector->y);

	//float d_ay = (current_acc_vector->y - previous_acc_vector->y);

	elevValue += (-p_p * p_ay);// + (d_r * d_ax);
}

void stabilizeYaw()
{
	float p_gz = (hover_gyro_vector->z - current_gyro_vector->z);

	ruddValue += -(-p_y * p_gz);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

