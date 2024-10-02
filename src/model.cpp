#include "model.h"

void Model::predict(Angle body_rate){

    //cache values for later
    sin_pitch = sinf(angle.pitch*PI/180);
    cos_pitch = cosf(angle.pitch*PI/180);
    sin_roll = sinf(angle.roll*PI/180);
    cos_roll = cosf(angle.roll*PI/180);
    tan_pitch = sin_pitch/cos_pitch;     //warning: null division (gymbal lock)

    // Use the current prediction in the calculation. Non linear 
    angle_rate.roll = body_rate.roll + body_rate.pitch*sin_roll*tan_pitch + body_rate.yaw*cos_roll*tan_pitch;
    angle_rate.pitch = body_rate.pitch*cos_roll - body_rate.yaw * sin_roll;
    angle_rate.yaw = body_rate.pitch*sin_roll/cos_pitch + body_rate.yaw*cos_roll/cos_pitch;

    angle.roll += DT * angle_rate.roll /1e3;
    angle.pitch += DT * angle_rate.pitch /1e3;
    angle.yaw += DT * angle_rate.yaw/1e3;  

    if (angle.yaw > 360){ angle.yaw -= 360; }
    else if (angle.yaw < 0) { angle.yaw += 360; }

}

Angle Model::euler_to_body(Angle euler_angle_rate){
    Angle body_angle_rate;

    body_angle_rate.roll = euler_angle_rate.roll - sin_pitch * euler_angle_rate.yaw;
    body_angle_rate.pitch = cos_roll * euler_angle_rate.pitch + sin_roll * cos_pitch * euler_angle_rate.yaw;
    body_angle_rate.yaw = -sin_roll * euler_angle_rate.pitch + cos_roll * cos_pitch * euler_angle_rate.yaw;

    return body_angle_rate;
}

void Model::coord_predict(Coords body_accel){
    // vehicle z-acceleration in cm/s^2
    double a_z = (1-(body_accel.x * sin_pitch - body_accel.y * cos_pitch * sin_roll + body_accel.z * cos_pitch * cos_roll - 0.03367 * abs(body_accel.x))) * 980 - 29;
    accel.z = a_z;
    veloc.z += a_z * DT * 1e-3;

}


