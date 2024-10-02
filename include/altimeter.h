#ifndef ALT 
#define ALT

#define ALTADDR 0x76
#include "Arduino.h"
#include "lpf.h"

class Altimeter {
    public:
        uint16_t dig_T1, dig_P1;
        int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
        int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
        float startalt = 0;
        void get_calibrations();
        void setup_altimeter();
        float read();
        void get_startalt();
        Lpf filter = Lpf(10, 50);
};


#endif