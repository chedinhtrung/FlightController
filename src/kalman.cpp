#include "kalman.h"
#include "math.h"
#include <Arduino.h>

void Kalman::predict(double new_rate, double &state){
    state += DT*new_rate/1000;
    P += Q;
}

void Kalman::bypass_predict(){
    P += Q;
}

void Kalman::correct(double new_meas, double &state){
    K = P/(P + R);
    state = state + K*(new_meas - state);
}

void Kalman::update(){
    P = (1-K)*(1-K)*P + K*K*R;
}

Kalman::Kalman(double flat_uncert, double gyro_uncert, double accel_uncert){
    P = flat_uncert*flat_uncert;
    Q = DT*DT*gyro_uncert*gyro_uncert/1e6;
    R = accel_uncert*accel_uncert;
}