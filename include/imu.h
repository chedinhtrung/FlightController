#ifndef IMU 
#define IMU

#include "config.h"
#include "datastructures.h"
#include <Arduino.h>

#define IMUADDR 0x68 // MPU6050 I2C address

struct ImuData {
    Angle angle;
    Angle angle_rate;
    Coords accel;
};

struct RawImuData {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

class Imu {
    public: 
        // Raw values for data gathering
        RawImuData raw_accels;   
        RawImuData raw_gyros;
        
        void setup();
        ImuData read();
        void calibrate();
        Angle angle_rate_offset;
        ImuData read_compensated(double roll, double pitch);
};

#endif 

