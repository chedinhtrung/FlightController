#include "ukf.h"

Ukf::Ukf(double flat_uncert, double gyro_uncert, double accel_uncert){
    P = flat_uncert*flat_uncert;
    Q = DT*DT*gyro_uncert*gyro_uncert/1e6 + 0.00001;
    R = accel_uncert*accel_uncert;
}

void Ukf::calculate_sigma(){
    for (int i=0; i<6; i++){ 
    }
}

void Ukf::correct(){

}

void Ukf::update(){
    
}