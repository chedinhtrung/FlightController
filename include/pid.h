#ifndef PIDCTL
#define PIDCTL
#include "lpf.h"

class PID {
    public:
        double P;
        double I;
        double D;
        double last_error = 0;
        double last_iterm = 0;
        //Lpf dfilter = Lpf(20.0);
        PID(double Pterm, double Iterm, double Dterm); 
        double calculate(double new_error);
        void reset();
};

#endif