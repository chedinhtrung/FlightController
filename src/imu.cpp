#include "imu.h"
#include "Wire.h"
#include <Arduino.h>

void Imu::setup(){
    // Wake up imu
    Wire.beginTransmission(IMUADDR);
    Wire.write(0x6B); 
    Wire.write(0x03);                   // Set PLL clock to use Z Axis Gyro as reference. also resets imu
    Wire.endTransmission();
    delay(100);

    // Configure Gyroscope
    Wire.beginTransmission(IMUADDR);         // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x1A);                        // Register 1A for Low Pass Filter (both gyro and accelerometer)
    Wire.write(0x04);                        // Set to 20Hz bandwidth
    Wire.endTransmission();
    delay(100);
 
    Wire.beginTransmission(IMUADDR);       
    Wire.write(0x1B);                       // Set sensitivity at Register 0x1B
    Wire.write(0x8);                        // Sensitivity at +- 500 degrees/s. 
    Wire.endTransmission();                 // IMPORTANT: This sets 65.5 bits/degree/s. Divide read value by 65.5!
    delay(100);

    // Configure Accelerometer
    Wire.beginTransmission(IMUADDR); 
    Wire.write(0x1C);                       // Set sensitivity at Register 0x1C
    Wire.write(0x08);                        // Set sensitivity at +-4g (non aggressive flying)
    Wire.endTransmission();
    calibrate();
}

ImuData Imu::read(){
    //unsigned long start = micros();
    ImuData data;

    // Request accelerometer data
    Wire.beginTransmission(IMUADDR);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(IMUADDR, 14);

    //Read accelerometer data as bits
    int16_t accelX = Wire.read()<<8 | Wire.read();
    int16_t accelY = Wire.read()<<8 | Wire.read();
    int16_t accelZ = Wire.read()<<8 | Wire.read();

    Wire.read(); Wire.read();      // Temperature data, not needed

    // Read gyro data
    int16_t gyroX = Wire.read()<<8 | Wire.read();
    int16_t gyroY = Wire.read()<<8 | Wire.read();
    int16_t gyroZ = Wire.read()<<8 | Wire.read();
    //Serial.println(micros() - start);

    // Calculate angular velocities  IMPORTANT: Roll and Pitch are assigned to Y and X respectively,
    // Because of how I mount my gyro 
    data.angle_rate.roll = (double)gyroY/65.5 - angle_rate_offset.roll;             // convention: right roll = positive
    data.angle_rate.pitch = ((double)gyroX/65.5 - angle_rate_offset.pitch);         // convention: up pitch = positive
    data.angle_rate.yaw = ((double)gyroZ/65.5 - angle_rate_offset.yaw) * -1.0;      // convention: right yaw = positive

    // put data to raw report

    raw_accels.x = accelX;
    raw_accels.y = accelY;
    raw_accels.z = accelZ;

    raw_gyros.x = gyroX;
    raw_gyros.y = gyroY;
    raw_gyros.z = gyroZ;

    // Calculate accelerometer data
    double AccX = (double)accelY/8192.0 - 0.0109;
    double AccY = (double)accelX/8192.0 + 0.01263;
    double AccZ = (double)accelZ/8192.0 - 0.2251;

    data.accel.x = AccX;
    data.accel.y = AccY;
    data.accel.z = AccZ;        

    // Convert to angles

    data.angle.roll = atan(-AccY/AccZ)/PI*180 - 0.4;                                // Convention: right roll = positive
    data.angle.pitch = atan(AccX/(sqrt(AccY*AccY + AccZ*AccZ)))/PI*180 + 0.2;       // Convention:  down = positive (mpu says up is positive!)
    
    return data;
}

void Imu::calibrate(){
    double RollRateOffset = 0;
    double PitchRateOffset = 0;
    double YawRateOffset = 0;

    /*
    double RollOffset = 0;
    double PitchOffset = 0;
    double YawOffset = 0;
    */
    for (int i=0; i<2000; i++){
        ImuData data = read();
        RollRateOffset += data.angle_rate.roll;
        PitchRateOffset += data.angle_rate.pitch;
        YawRateOffset += data.angle_rate.yaw;
        delay(1);
        /*
        RollOffset += data.MRoll;
        PitchOffset += data.MPitch;
        YawOffset += data.MYaw;
        */
    }


    angle_rate_offset.roll = RollRateOffset/2000;       // convention: right roll = positive
    angle_rate_offset.pitch = PitchRateOffset/2000;    // convention: up pitch = positive
    angle_rate_offset.yaw = -YawRateOffset/2000;        // convention: right yaw = positive

    /*
    PitchOffset /= 10000;
    RollOffset/=10000;
    YawOffset /= 10000;
    Serial.print("RollOffset: ");
    Serial.print(RollOffset, 6);
    Serial.print(" PitchOffset: ");
    Serial.print(PitchOffset, 6);
    Serial.print(" YawOffset: ");
    Serial.print(YawOffset, 6);
    */
}