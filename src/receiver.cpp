#include "receiver.h"

Receiver::Receiver(){
    in.begin(9);            // Use pin 6 for PPM Input
}

ReceiverData Receiver::recv(){
    ReceiverData data;
    data.ThrottleIn = in.read(3);       // Channel 3 = throttle
    data.RollIn = in.read(4);           // Channel 4 = roll
    data.YawIn = in.read(1);            // Channel 1 = yaw
    data.PitchIn = in.read(2);          // Channel 2 = pitch
    data.AuxChannel5In = in.read(5);    // Aux Channel 5
    data.AuxChannel6In = in.read(6);    // Aux Channel 6

    // Prevent zeroes at startup

    if (data.ThrottleIn < 1000){data.ThrottleIn = 1000;}
    else if (data.ThrottleIn > 2000){data.ThrottleIn = 2000;}

    if (data.RollIn < 900){data.RollIn = 1500;}
    else if (data.RollIn > 2000){data.RollIn = 2000;}

    if (data.PitchIn < 900){data.PitchIn = 1500;}
    else if (data.PitchIn > 2000){data.PitchIn = 2000;}

    if (data.YawIn < 900){data.YawIn = 1500;}
    else if (data.YawIn > 2000){data.YawIn = 2000;}
    
    return data;
}

ReceiverData Receiver::to_anglemode(ReceiverData data){

    // convert controller input to desired angle 

    data.PitchIn = (-data.PitchIn+1500.0)/1000.0*70.0;       // map 1000us, 2000us ---> -35,35 degree
    data.RollIn = (data.RollIn-1500.0)/1000.0*70.0;         // map 1000us, 2000us ---> -35,35 degree
    data.YawIn = (data.YawIn-1500.0)/1000.0*60.0 - 0.06;           // allow up to +- 30 degree/s rate on yaw

    data.ThrottleIn = (data.ThrottleIn - 1000)/1000;        // map 1000us, 2000us --> 0, 1 throttle percentage
    
    if (data.ThrottleIn > 1){ data.ThrottleIn = 1;}
    else if (data.ThrottleIn < 0) {data.ThrottleIn = 0;}    // Prevent below 0/ above 1 throttle

    if (data.PitchIn > 35){data.PitchIn = 35;}              // Prevent overshooting
    else if (data.PitchIn < -35){data.PitchIn = -35;}

    if (data.RollIn > 35){data.RollIn = 35;}
    else if (data.RollIn < -35){data.RollIn = -35;}

    if (data.YawIn > 30){data.YawIn = 30;}
    else if (data.YawIn < -30){data.RollIn = -30;}


    return data;

}