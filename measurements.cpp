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

    // Account for offsets
    ax += -1*ax_off;
    ay += -1*ay_off;
    az += -1*az_off;

    gx += -1*gx_off;
    gy += -1*gy_off;
    gz += -1*gz_off;

    // display tab-separated accel/gyro x/y/z values
    // Serial.print("ax: "); Serial.print(ax); Serial.print(" \t");
    // Serial.print("ay: "); Serial.print(ay); Serial.print(" \t");
    // Serial.print("az: "); Serial.print(az); Serial.print(" \t");
    // Serial.print("gx: "); Serial.print(gx); Serial.print(" \t");
    // Serial.print("gy: "); Serial.print(gy); Serial.print("  \t");
    // Serial.print("gz: "); Serial.println(gz);

    // Translate into terms of g.
    raw_measurements[0] = ((float)ax);
    raw_measurements[1] = ((float)ay);
    raw_measurements[2] = ((float)az);

    // Translate into terms of degrees/s
    raw_measurements[3] = ((float)gx);
    raw_measurements[4] = ((float)gy);
    raw_measurements[5] = ((float)gz);

    for(int i = 0; i < 3; i++)
    {
        translated_measurements[i] = raw_measurements[i] / ACC_SENSITIVITY;
    }

    for(int i = 3; i < 6; i++)
    {
        translated_measurements[i] = raw_measurements[i] / GYRO_SENSITIVITY;
    }

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

    // Serial.print("ax: "); Serial.print(sum_acceleration_x); Serial.print(" \t");
    // Serial.print("ay: "); Serial.print(sum_acceleration_y); Serial.print(" \t");
    // Serial.print("az: "); Serial.print(sum_acceleration_z); Serial.print(" \t");
    // Serial.print("gx: "); Serial.print(sum_gyro_x); Serial.print(" \t");
    // Serial.print("gy: "); Serial.print(sum_gyro_y); Serial.print("  \t");
    // Serial.print("gz: "); Serial.println(sum_gyro_z);
}
