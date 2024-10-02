#ifndef UKF
#define UKF 

#include "config.h"

class Ukf {
    public:
        double Pk = 0;
        
        double P = 4.0;                     // initial P = flat surface uncertainty = 2 degrees^2
        double Q = DT*DT*1.5*1.5/1000000;   // Gyro uncertainty = 1.5 degree/s 
        double R = 1;                       // Accelerometer angular uncertainty = 1 degree (at stationary)
        double K = 0;

        double sigmas[6];                       
        
        Ukf(double flat_uncert, double gyro_uncert, double accel_uncert);
        void calculate_sigma();
        void update();
        void correct();
};

#endif
