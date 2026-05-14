An Arduino flight controller for Teensy 4.0 inspired by Carbon Aeronautics. 
Main improvements: 
- Correct the handling of rotation, coordinate transform and Kalman filtering for much more stable flight
- Non-linear control by simply swapping out the P in PID for a square-root curve
- Log data to Raspberry Pi flight computer via Serial

See it fly: https://www.youtube.com/watch?v=vWL1y871umU
