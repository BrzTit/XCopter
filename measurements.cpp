#include "measurements.h"


// DATA DECLARATIONS
// ===================================================
MPU6050 accelgyro;

// Offset values
int16_t gx_off, gy_off, gz_off, ax_off, ay_off, az_off;

float measurements[6];
float calculations[2];

// ====================================================

void updateIMUValues()
{

    int16_t ax, ay, az, gx, gy, gz;

	// read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Account for offsets
    ax += -1*ax_off;
    ay += -1*ay_off;
    az += -1*az_off;

    gx += -1*gx_off;
    gy += -1*gy_off;
    gz += -1*gz_off;

    // Translate into terms of g.
    measurements[0] = (float)ax/ACC_SENSITIVITY;
    measurements[1] = (float)ay/ACC_SENSITIVITY;
    measurements[2] = (float)az/ACC_SENSITIVITY;

    // Translate into terms of degrees/s
    measurements[3] = (float)gx/GYRO_SENSITIVITY;
    measurements[4] = (float)gy/GYRO_SENSITIVITY;
    measurements[5] = (float)gz/GYRO_SENSITIVITY;

}

float calcRollAngle()
{
    return -1;
}

float calcPitchAngle()
{
    return -1;
}

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
    float sum_acceleration_x = 0, sum_acceleration_y = 0, sum_acceleration_z = 0;
    float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;

    Serial.println("Summing IMU Values...");
    for(int i = 0; i<1000;i++)
    {
        updateIMUValues();

        sum_acceleration_x += measurements[0];
        sum_acceleration_y += measurements[1];
        sum_acceleration_z += measurements[2];

        sum_gyro_x += measurements[3];
        sum_gyro_y += measurements[4];
        sum_gyro_z += measurements[5];
    }
    Serial.println("Finished summing...");

    Serial.println("Averaging Total Values...");
    ax_off = (int)(sum_acceleration_x/1000.0);
    ay_off = (int)(sum_acceleration_y/1000.0);
    az_off = (int)(sum_acceleration_z/1000.0) - ACC_SENSITIVITY;

    Serial.println("Applying offsets...");
    gx_off = (int)(sum_gyro_x/1000.0);
    gy_off = (int)(sum_gyro_y/1000.0);
    gz_off = (int)(sum_gyro_z/1000.0);
}
