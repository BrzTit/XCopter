#include "measurements.h"


// DATA DECLARATIONS
// ===================================================
MPU6050 accelgyro;
MS5611 barometer;
HMC5883L compass;

// Offset values
int16_t gx_off = 0, gy_off = 0, gz_off = 0, ax_off = 0, ay_off = 0, az_off = 0;

// Measurement buffers
float raw_measurements[NUM_MEASUREMENTS];
float translated_measurements[NUM_MEASUREMENTS];
float buffered_measurements[NUM_MEASUREMENTS][BUFFER_SIZE];
float filtered_measurements[NUM_MEASUREMENTS];
float calculations[3];

float referencePressure = 0;

// Kalman filter constants
float q1 = 0.01, q2 = 0.01, q3 = 0.001;
float r1 = 0.035697, r2 = 0.013996;

float q1_yaw = 0.01, q2_yaw = 0.01, q3_yaw = 0.001;
float r1_yaw = 0, r2_yaw = 0;

// Kalman filter matrices
Eigen::Matrix<float, 3, 1> X_roll, X_pitch, X_yaw; 
Eigen::Matrix3f A_roll, A_pitch, A_yaw;
Eigen::Matrix3f Q_roll, Q_pitch, Q_yaw;
Eigen::Matrix3f P_roll, P_pitch, P_yaw;
Eigen::Matrix<float, 2, 3> H_roll, H_pitch; 
Eigen::Matrix<float, 1, 3> H_yaw;
Eigen::Matrix2f R_roll, R_pitch; 
Eigen::Matrix<float, 1, 1> R_yaw;
Eigen::Matrix3f I;

Eigen::Matrix<float, 2, 1> Z_roll, Z_pitch;
Eigen::Matrix<float, 1, 1> Z_yaw;
Eigen::Matrix<float, 2, 1> Y_roll, Y_pitch;
Eigen::Matrix<float, 1, 1> Y_yaw;
Eigen::Matrix2f S_roll, S_pitch;
Eigen::Matrix<float, 1, 1> S_yaw;
Eigen::Matrix<float, 3, 2> K_roll, K_pitch;
Eigen::Matrix<float, 3, 1> K_yaw;

// ====================================================

void initKalmanValues()
{
    // Initial orientation, rate, and bias
    X_roll(0,0) = 0;  
    X_roll(1,0) = 0;  
    X_roll(2,0) = 0; 

    X_pitch(0,0) = 0;
    X_pitch(1,0) = 0;
    X_pitch(2,0) = 0;

    X_yaw(0,0) = 0;
    X_yaw(1,0) = 0;
    X_yaw(2,0) = 0;

    // State transition matrix
    A_roll(0,0) = 1; A_roll(0,1) = IMUPollPeriodSec; A_roll(0,2) = -IMUPollPeriodSec; 
    A_roll(1,0) = 0; A_roll(1,1) = 1; A_roll(1,2) = 0; 
    A_roll(2,0) = 0; A_roll(2,1) = 0; A_roll(2,2) = 1; 

    A_pitch(0,0) = 1; A_pitch(0,1) = IMUPollPeriodSec; A_pitch(0,2) = -IMUPollPeriodSec;
    A_pitch(1,0) = 0; A_pitch(1,1) = 1; A_pitch(1,2) = 0;
    A_pitch(2,0) = 0; A_pitch(2,1) = 0; A_pitch(2,2) = 1;

    A_yaw(0,0) = 1; A_yaw(0,1) = IMUPollPeriodSec; A_yaw(0,2) = -IMUPollPeriodSec;
    A_yaw(1,0) = 0; A_yaw(1,1) = 1; A_yaw(1,2) = 0;
    A_yaw(2,0) = 0; A_yaw(2,1) = 0; A_yaw(2,2) = 1;


    // Process noise covariance matrix
    Q_roll(0,0) = q1; Q_roll(0,1) = 0; Q_roll(0,2) = 0; 
    Q_roll(1,0) = 0; Q_roll(1,1) = q2; Q_roll(1,2) = 0; 
    Q_roll(2,0) = 0; Q_roll(2,1) = 0; Q_roll(2,2) = q3; 

    Q_pitch(0,0) = q1; Q_pitch(0,1) = 0; Q_pitch(0,2) = 0;
    Q_pitch(1,0) = 0; Q_pitch(1,1) = q2; Q_pitch(1,2) = 0;
    Q_pitch(2,0) = 0; Q_pitch(2,1) = 0; Q_pitch(2,2) = q3;

    Q_yaw(0,0) = q1_yaw; Q_yaw(0,1) = 0; Q_yaw(0,2) = 0;
    Q_yaw(1,0) = 0; Q_yaw(1,1) = q2_yaw; Q_yaw(1,2) = 0;
    Q_yaw(2,0) = 0; Q_yaw(2,1) = 0; Q_yaw(2,2) = q3_yaw;

    // State Estimate
    P_roll(0,0) = 0; P_roll(0,1) = 0; P_roll(0,2) = 0; 
    P_roll(1,0) = 0; P_roll(1,1) = 0; P_roll(1,2) = 0; 
    P_roll(2,0) = 0; P_roll(2,1) = 0; P_roll(2,2) = 0;

    P_pitch(0,0) = 0; P_pitch(0,1) = 0; P_pitch(0,2) = 0;
    P_pitch(1,0) = 0; P_pitch(1,1) = 0; P_pitch(1,2) = 0;
    P_pitch(2,0) = 0; P_pitch(2,1) = 0; P_pitch(2,2) = 0;

    P_yaw(0,0) = 0; P_yaw(0,1) = 0; P_yaw(0,2) = 0;
    P_yaw(1,0) = 0; P_yaw(1,1) = 0; P_yaw(1,2) = 0;
    P_yaw(2,0) = 0; P_yaw(2,1) = 0; P_yaw(2,2) = 0;

    // Measurement transition matrix
    H_roll(0,0) = 1; H_roll(0,1) = 0; H_roll(0,2) = 0;
    H_roll(1,0) = 0; H_roll(1,1) = 1; H_roll(1,2) = 0;

    H_pitch(0,0) = 1; H_pitch(0,1) = 0; H_pitch(0,2) = 0;
    H_pitch(1,0) = 0; H_pitch(1,1) = 1; H_pitch(1,2) = 0;

    H_yaw(0,0) = 0; H_yaw(0,1) = 1; H_yaw(0,2) = 0;

    // Measurement noise covariance matrix
    R_roll(0,0) = r1; R_roll(0,1) = 0;
    R_roll(1,0) = 0; R_roll(1,1) = r2;

    R_pitch(0,0) = r1; R_pitch(0,1) = 0;
    R_pitch(1,0) = 0; R_pitch(1,1) = r2;

    R_yaw(0,0) = r2;
    // R_yaw(0,0) = r1_yaw; R_yaw(0,1) = 0;
    // R_yaw(1,0) = 0; R_yaw(1,1) = r2_yaw;

    // Identity Matrix
    I(0,0) = 1; I(0,1) = 0; I(0,2) = 0;
    I(1,0) = 0; I(1,1) = 1; I(1,2) = 0;
    I(2,0) = 0; I(2,1) = 0; I(2,2) = 1;

}

void calcRollAngle()
{
    // Z faces up and out of quad
    // X faces to the right of the quad
    // Y faces forward of the quad

    // Angle is given by geometry of gravity vector
    // acc_angle = atan2(-x, -z);
    float acc_angle = atan2(-filtered_measurements[0], -filtered_measurements[2]) * (180.0 / M_PI);

    // Serial.print(acc_angle);
    // Serial.print(", ");
    // Serial.print(filtered_measurements[4]);
    // Serial.println(",");

    // Update the measurements
    Z_roll(0,0) = acc_angle;
    Z_roll(1,0) = filtered_measurements[4];

    // Predict
    X_roll = A_roll*X_roll;
    P_roll = A_roll * P_roll * A_roll.transpose() + Q_roll;

    // Update
    Y_roll = Z_roll - H_roll * X_roll;
    S_roll = H_roll * P_roll * H_roll.transpose() + R_roll;
    K_roll = P_roll * H_roll.transpose() * S_roll.inverse();
    X_roll = X_roll  + K_roll * Y_roll;
    P_roll = (I-K_roll * H_roll) * P_roll;

    calculations[0] = X_roll(0,0);
    Serial.print(calculations[0]);
    Serial.print("\t");

}

void calcPitchAngle()
{
    // Z faces up and out of quad
    // X faces to the right of the quad
    // Y faces forward of the quad

    // Angle is given by geometry of gravity vector
    // acc_angle = atan2(-x, -z);
    float acc_angle = atan2(-filtered_measurements[1], -filtered_measurements[2]) * (180.0 / M_PI);

    // Update the measurements
    Z_pitch(0,0) = acc_angle;
    Z_pitch(1,0) = filtered_measurements[3];

    // Predict
    X_pitch = A_pitch*X_pitch;
    P_pitch = A_pitch * P_pitch * A_pitch.transpose() + Q_pitch;

    // Update
    Y_pitch = Z_pitch - H_pitch * X_pitch;
    S_pitch = H_pitch * P_pitch * H_pitch.transpose() + R_pitch;
    K_pitch = P_pitch * H_pitch.transpose() * S_pitch.inverse();
    X_pitch = X_pitch  + K_pitch * Y_pitch;
    P_pitch = (I-K_pitch * H_pitch) * P_pitch;

    calculations[1] = X_pitch(0,0);
    Serial.println(calculations[1]);
}

void calcYawAngle()
{
    // Z faces up and out of quad
    // X faces to the right of the quad
    // Y faces forward of the quad
    Z_yaw(0,0) = filtered_measurements[5];

    // Predict
    X_yaw = A_yaw*X_yaw;
    P_yaw = A_yaw * P_yaw * A_yaw.transpose() + Q_yaw;

    // Update
    Y_yaw = Z_yaw - H_yaw * X_yaw;
    S_yaw = H_yaw * P_yaw * H_yaw.transpose() + R_yaw;
    K_yaw = P_yaw * H_yaw.transpose() * S_yaw.inverse();
    X_yaw = X_yaw  + K_yaw * Y_yaw;
    P_yaw = (I-K_yaw * H_yaw) * P_yaw;

    calculations[2] = X_yaw(0,0);
    Serial.println(calculations[2]);
}


void initializeSensors()
{
    initializeIMU();
    initializeBarometer();
    initializeCompass();
}

void initializeIMU()
{
    Serial.println("Initializing kalman values...");
    initKalmanValues();

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
}

void initializeBarometer()
{
    Serial.println("Initialized the barometer...");
    while(!barometer.begin())
    {
        Serial.println("Could not find a valid MS5611 sensor, check wiring!");
        delay(100);
    }

    // Get reference pressure for relative altitude
    referencePressure = barometer.readPressure(true);

    Serial.println("Initialized.");
}

void initializeCompass()
{

    // Initialize HMC5883L
    Serial.println("Initialize HMC5883L");
    while (!compass.begin())
    {
        Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
        delay(100);
    }

    // Set measurement range
    // +/- 0.88 Ga: HMC5883L_RANGE_0_88GA
    // +/- 1.30 Ga: HMC5883L_RANGE_1_3GA (default)
    // +/- 1.90 Ga: HMC5883L_RANGE_1_9GA
    // +/- 2.50 Ga: HMC5883L_RANGE_2_5GA
    // +/- 4.00 Ga: HMC5883L_RANGE_4GA
    // +/- 4.70 Ga: HMC5883L_RANGE_4_7GA
    // +/- 5.60 Ga: HMC5883L_RANGE_5_6GA
    // +/- 8.10 Ga: HMC5883L_RANGE_8_1GA
    compass.setRange(HMC5883L_RANGE_1_3GA);

    // Set measurement mode
    // Idle mode:              HMC5883L_IDLE
    // Single-Measurement:     HMC5883L_SINGLE
    // Continuous-Measurement: HMC5883L_CONTINOUS (default)
    compass.setMeasurementMode(HMC5883L_CONTINOUS);

    // Set data rate
    //  0.75Hz: HMC5883L_DATARATE_0_75HZ
    //  1.50Hz: HMC5883L_DATARATE_1_5HZ
    //  3.00Hz: HMC5883L_DATARATE_3HZ
    //  7.50Hz: HMC5883L_DATARATE_7_50HZ
    // 15.00Hz: HMC5883L_DATARATE_15HZ (default)
    // 30.00Hz: HMC5883L_DATARATE_30HZ
    // 75.00Hz: HMC5883L_DATARATE_75HZ
    compass.setDataRate(HMC5883L_DATARATE_15HZ);

    // Set number of samples averaged
    // 1 sample:  HMC5883L_SAMPLES_1 (default)
    // 2 samples: HMC5883L_SAMPLES_2
    // 4 samples: HMC5883L_SAMPLES_4
    // 8 samples: HMC5883L_SAMPLES_8
    compass.setSamples(HMC5883L_SAMPLES_1);
    Serial.println("Initialized.");

}

void updateSensors()
{
    updateIMUValues();
    updateBarometerValues();
    updateCompassValues();
    filterMeasurements();
}

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

}

void updateBarometerValues()
{
    // Read true temperature & Pressure
    double realTemperature = barometer.readTemperature(true);
    long realPressure = barometer.readPressure(true);

    // Calculate altitude
    float absoluteAltitude = barometer.getAltitude(realPressure);
    float relativeAltitude = barometer.getAltitude(realPressure, referencePressure);

    raw_measurements[7] = relativeAltitude;
    translated_measurements[7] = relativeAltitude;
}

void updateCompassValues()
{
    Vector raw = compass.readRaw();
    Vector norm = compass.readNormalize();

    Serial.print(" Xraw = ");
    Serial.print(raw.XAxis);
    Serial.print(" Yraw = ");
    Serial.print(raw.YAxis);
    Serial.print(" Zraw = ");
    Serial.print(raw.ZAxis);
    Serial.print(" Xnorm = ");
    Serial.print(norm.XAxis);
    Serial.print(" Ynorm = ");
    Serial.print(norm.YAxis);
    Serial.print(" ZNorm = ");
    Serial.print(norm.ZAxis);
    Serial.println();  

    delay(100);
}

// Moving average filter for both gyro and accelerometer
void filterMeasurements()
{
    // Circular buffer counter
    static int counter = 0;

    // Move all the raw measurements into the circular buffer
    for(int i = 0; i < NUM_MEASUREMENTS; i++)
    {
        buffered_measurements[i][counter] = translated_measurements[i];
    }

    // Average the measurements in the buffer and stick them into the filtered measurement array
    float avg[NUM_MEASUREMENTS] = {0, 0, 0, 0, 0, 0, 0};
    for(int p = 0; p < NUM_MEASUREMENTS; p++)
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
