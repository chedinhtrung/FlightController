#ifndef MOTOR 
#define MOTOR
#include <Arduino.h>

struct RawMotor {
    uint16_t fl;
    uint16_t fr;
    uint16_t bl;
    uint16_t br;
};

class Motor {
    public:
        RawMotor motor_report;
        void set_motor(int m1, int m2, int m3, int m4);
        void set_motor(float fl, float fr, float bl, float br);
};

/*
void set_motor(int m1, int m2, int m3, int m4);
void set_motor(float fl, float fr, float bl, float br);
*/

#endif 