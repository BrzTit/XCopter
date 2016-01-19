#include "measurements.h"


// DATA DECLARATIONS
// ===================================================
MPU6050 accelgyro;

// Offset values
int16_t gx_off = 0, gy_off = 0, gz_off = 0, ax_off = 0, ay_off = 0, az_off = 0;

float raw_measurements[6];
float translated_measurements[6];
float buffered_measurements[6][BUFFER_SIZE];
float filtered_measurements[6];
float calculations[2];

// ====================================================

void updateIMUValues()
{

    int16_t ax, ay, az, gx, gy, gz;

	// read raw accel/gyro raw_measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    raw_measurements[0] = ((float)ax);
    raw_measurements[1] = ((float)ay);
    raw_measurements[2] = ((float)az);

    raw_measurements[3] = ((float)gx);
    raw_measurements[4] = ((float)gy);
    raw_measurements[5] = ((float)gz);

    // Translate into terms of g.
    for(int i = 0; i < 3; i++)
    {
        translated_measurements[i] = raw_measurements[i] / ACC_SENSITIVITY;
    }

    // Invert the Z axis
    translated_measurements[2] = -translated_measurements[2];

    // Translate into terms of degrees/s
    for(int i = 3; i < 6; i++)
    {
        translated_measurements[i] = raw_measurements[i] / GYRO_SENSITIVITY;
    }
    filterMeasurements();
}

// Moving average filter for both gyro and accelerometer
void filterMeasurements()
{
    // Circular buffer counter
    static int counter = 0;

    // Move all the raw measurements into the circular buffer
    for(int i = 0; i < 6; i++)
    {
        buffered_measurements[i][counter] = translated_measurements[i];
    }

    // Average the measurements in the buffer and stick them into the filtered measurement array
    float avg[6] = {0, 0, 0, 0, 0, 0};
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
    while(calibrateIMU() == false);
    Serial.println("Finished calibration.");
        delay(5000);
    
}

// Takes an average of 100 readings from the IMU and averages them to get the offset.
bool calibrateIMU()
{
    float sum_acceleration_x = 0, sum_acceleration_y = 0, sum_acceleration_z = 0;
    float sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;

    Serial.println("Summing IMU Values...");
    for(int i = 0; i<500;i++)
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
    ax_off = ((int)((sum_acceleration_x/500.0) / 8.0));
    ay_off = ((int)((sum_acceleration_y/500.0) / 8.0));
    az_off = (int)((sum_acceleration_z/500.0 - ACC_SENSITIVITY) / 8.0);

    Serial.println("Applying offsets...");
    gx_off = (int)((sum_gyro_x/500.0) / 4.0);
    gy_off = (int)((sum_gyro_y/500.0) / 4.0);
    gz_off = (int)((sum_gyro_z/500.0) / 4.0);

    accelgyro.setXAccelOffset(accelgyro.getXAccelOffset() - ax_off); 
    accelgyro.setYAccelOffset(accelgyro.getYAccelOffset() - ay_off); 
    accelgyro.setZAccelOffset(accelgyro.getZAccelOffset() - az_off); 

    accelgyro.setXGyroOffset(accelgyro.getXGyroOffset() - gx_off); 
    accelgyro.setYGyroOffset(accelgyro.getYGyroOffset() - gy_off); 
    accelgyro.setZGyroOffset(accelgyro.getZGyroOffset() - gz_off);

    // Test the offset values
    Serial.println("Testing...");

    sum_acceleration_x = 0, sum_acceleration_y = 0, sum_acceleration_z = 0;
    sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;

    for(int i = 0; i < 500; i ++)
    {
        sum_acceleration_x += raw_measurements[0];
        sum_acceleration_y += raw_measurements[1];
        sum_acceleration_z += raw_measurements[2];

        sum_gyro_x += raw_measurements[3];
        sum_gyro_y += raw_measurements[4];
        sum_gyro_z += raw_measurements[5];
    }

    ax_off = (int)(sum_acceleration_x/500.0);
    ay_off = (int)(sum_acceleration_y/500.0);
    az_off = (int)(sum_acceleration_z/500.0 - ACC_SENSITIVITY);

    gx_off = (int)(sum_gyro_x/500.0);
    gy_off = (int)(sum_gyro_y/500.0);
    gz_off = (int)(sum_gyro_z/500.0);

    // 0.03g or .22 deg/s
    int accel_tolerance = 250, gyro_tolerance = 15;

    return (abs(ax_off) < accel_tolerance) && (abs(ay_off) < accel_tolerance) && (abs(az_off) < accel_tolerance)
    && (abs(gx_off) < gyro_tolerance) && (abs(gy_off) < gyro_tolerance) && (abs(gz_off) < gyro_tolerance);


}
