#include "lpf.h"

Lpf::Lpf(double bandwidth, double st) {
    sample_time = st;
    double tau = 1/(2*3.14159265358*bandwidth);
    alpha = sample_time*1e-3 / (sample_time*1e-3 + tau);
}

Lpf::Lpf(double bandwidth) {
    sample_time = DT;
    double tau = 1/(2*3.14159265358*bandwidth);
    alpha = sample_time*1e-3 / (sample_time*1e-3 + tau);
}

double Lpf::filter(double value){
    filtered = alpha * value + (1.0 - alpha)*filtered;
    double val = filtered;
    return val;
}

void Lpf::reset(){
    filtered = 0.0;
}