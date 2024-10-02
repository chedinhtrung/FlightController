#include "pid.h"
#include "config.h"

PID::PID(double Pterm, double Iterm, double Dterm){
    P = Pterm;
    I = Iterm;
    D = Dterm;
}

double PID::calculate(double new_error) {
    double pterm = P*new_error;
    
    last_iterm += I*0.5*(new_error + last_error)*DT*1e-3;     // trapezoidal integration
    last_error = new_error;
    if (last_iterm > 0.08){
        last_iterm = 0.08;                                // Anti integrator windup. cap i_term at 20% throttle adjustment
    }
    else if (last_iterm < -0.08){
        last_iterm = -0.08;
    }
    double dterm = D*(new_error - last_error)/(DT*1e-3);
    if (dterm > 0.12) {dterm = 0.12;}
    else if (dterm < -0.12){ dterm = -0.12;}
    double pid_out = pterm + dterm + last_iterm;
    if (pid_out > 0.4){
        pid_out = 0.4;          // PID limit at 30% motor power
    }
    if (pid_out < -0.4){
        pid_out = -0.4;          // PID limit at 30% motor power
    }
    return pid_out;
}

void PID::reset(){
    last_error = 0.0;
    last_iterm = 0.0;
    //dfilter.reset();
}