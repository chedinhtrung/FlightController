#ifndef LPF 
#define  LPF
#include "config.h"

class Lpf { 
    public: 
        Lpf(double bandwidth);
        Lpf(double bandwidth, double st);
        double sample_time = DT;
        double alpha;
        double filtered = 0.0;
        double filter(double value);
        void reset();
};

#endif