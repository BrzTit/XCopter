#include "measurements.h"


// DATA DECLARATIONS
// ===================================================
MPU6050 accelgyro;

// Offset values
int16_t gx_off, gy_off, gz_off, ax_off, ay_off, az_off;

float raw_measurements[6];
float buffered_measurements[6][BUFFER_SIZE];
float filtered_measurements[6];
float calculations[2];

// ====================================================

void updateIMUValues()
{

    int16_t ax, ay, az, gx, gy, gz;

	// read raw accel/gyro raw_measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Account for offsets
    ax += -1*ax_off;
    ay += -1*ay_off;
    az += -1*az_off;

    gx += -1*gx_off;
    gy += -1*gy_off;
    gz += -1*gz_off;

    // Translate into terms of g.
    raw_measurements[0] = (float)ax/ACC_SENSITIVITY;
    raw_measurements[1] = (float)ay/ACC_SENSITIVITY;
    raw_measurements[2] = (float)az/ACC_SENSITIVITY;

    // Translate into terms of degrees/s
    raw_measurements[3] = (float)gx/GYRO_SENSITIVITY;
    raw_measurements[4] = (float)gy/GYRO_SENSITIVITY;
    raw_measurements[5] = (float)gz/GYRO_SENSITIVITY;

}

// Moving average filter for both gyro and accelerometer
void filterMeasurements()
{
    // Circular buffer counter
    static int counter = 0;

    // Move all the raw measurements into the circular buffer
    for(int i = 0; i < 6; i++)
    {
        buffered_measurements[i][counter] = raw_measurements[i];
    }

    // Average the measurements in the buffer and stick them into the filtered measurement array
    float avg[6];
    for(int p = 0; p < 6; p++)
    {
        for(int i = 0; i < BUFFER_SIZE; i++)
        {
            avg[p] += buffered_measurements[p][i];
        }
        filtered_measurements[p] = avg[p] / (float)BUFFER_SIZE;
    }

    // Incrememnt and roll over the circular buffer counter
    counter += 1;
    if(counter == BUFFER_SIZE)
        {counter = 0;}
}

void calcRollAngle()
{
    static float gyro_angle = 0, acc_angle = 0;

    // Angle is rate * dt 
    gyro_angle += filtered_measurements[4] * IMUPollPeriodSec;

    // Z faces up and out of quad
    // X faces to the right of the quad
    // Y faces forward of the quad

    // Angle is given by geometry of gravity vector
    // acc_angle = atan2(-x, -z);
    acc_angle = atan2(-filtered_measurements[0], -filtered_measurements[2]) * (180.0 / M_PI);

    calculations[0] = GYRO_PERCENTAGE * gyro_angle + ACC_PERCENTAGE * acc_angle;
}

void calcPitchAngle()
{
    static float gyro_angle = 0, acc_angle = 0;

    // Angle is rate * dt 
    gyro_angle += filtered_measurements[3] * IMUPollPeriodSec;

    // Z faces up and out of quad
    // X faces to the right of the quad
    // Y faces forward of the quad

    // Angle is given by geometry of gravity vector
    // acc_angle = atan2(-y, -z);
    acc_angle = atan2(-filtered_measurements[1], -filtered_measurements[2]) * (180.0 / M_PI);

    calculations[1] = GYRO_PERCENTAGE * gyro_angle + ACC_PERCENTAGE * acc_angle;
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

        sum_acceleration_x += raw_measurements[0];
        sum_acceleration_y += raw_measurements[1];
        sum_acceleration_z += raw_measurements[2];

        sum_gyro_x += raw_measurements[3];
        sum_gyro_y += raw_measurements[4];
        sum_gyro_z += raw_measurements[5];
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
