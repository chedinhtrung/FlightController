#ifndef MODEL 
#define MODEL
#include "math.h"
#include "Arduino.h"
#include "config.h"
#include "datastructures.h"

class Model {
    public: 
        Angle angle;
        Angle angle_rate;
        Coords coord;
        Coords veloc;
        Coords accel;
        // Cache trigonometric values for further usage 
        double sin_pitch = sinf(angle.pitch*PI/180);
        double cos_pitch = cosf(angle.pitch*PI/180);
        double sin_roll = sinf(angle.roll*PI/180);
        double cos_roll = cosf(angle.roll*PI/180);
        double tan_pitch = sin_pitch/cos_pitch;

        void predict(Angle body_rate);
        void coord_predict(Coords accel);
        Angle euler_to_body(Angle euler_angle_rate);
        Coords body_to_inert(Coords body_accel);

};

#endif