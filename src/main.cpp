
#include <Wire.h>
#include "datastructures.h"
#include "motor.h"
#include "imu.h"
#include "dps310.h"
#include "kalman.h"
#include "model.h"
#include "pid.h"
#include <PulsePosition.h>
#include "receiver.h"
#include "TinyGPS++.h"

#include <Servo.h>

Servo gimbal; 

#define STARTBYTE 0x02

static Model model;
static Report report;

Kalman rollkalmanfilter = Kalman(1.5, 2, 4);
Kalman pitchkalmanfilter = Kalman(1.5, 2, 4);

PulsePositionInput control_in(RISING);
Receiver receiver = Receiver();
Imu imu = Imu();
Dps310Altimeter alt = Dps310Altimeter();
TinyGPSPlus gps;

PID pitchratepid = PID(0.000137, 0.000168, 0.00198*1e-3); 
PID rollratepid = PID(0.00017, 0.000168, 0.0022*1e-3); 
PID yawratepid = PID(0.0018, 0.00018, 0);
//PID anglepid = PID();
/*
//Baseline
PID pitchratepid = PID(0.000137, 0.00015, 0.00204*1e-3); 
PID rollratepid = PID(0.000137, 0.00015, 0.00204*1e-3); 
PID yawratepid = PID(0.0018, 0.00018, 0);
*/

Motor motor;

unsigned long last_active = micros();
unsigned long last_compass = micros();
unsigned long last_report = micros();
unsigned long last_altupdate = micros();
unsigned long last_gimbal = micros();

double h_alpha = 0.97;
double h = 0;
double vert_speed = 0;

void clearI2C() {
  Wire.begin();
  
  // Toggle SCL and SDA pins to clear any stuck state
  pinMode(SCL, OUTPUT);
  pinMode(SDA, OUTPUT);
  
  // Generate 9 clock pulses to reset any stuck devices
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delay(1);
    digitalWrite(SCL, HIGH);
    delay(1);
  }
  
  // Send a stop signal
  digitalWrite(SDA, LOW);
  delay(1);
  digitalWrite(SCL, HIGH);
  delay(1);
  digitalWrite(SDA, HIGH);

}


void setup() {
  gimbal.attach(A8);
  gimbal.write(10);

  delay(5000);
  analogWriteFrequency(2, 2000);
  analogWriteFrequency(3, 2000);
  analogWriteFrequency(4, 2000);
  analogWriteFrequency(5, 2000);
  analogWriteResolution(12);

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial4.begin(115200);

  //clearI2C();
  delay(100);
  
  //while (!Serial1){}                    // block until RPi goes online
  Wire.begin();
  Wire.setClock(400000);
  
  delay(4000);
  imu.setup();
  delay(200);
  //comp.setup();
  //delay(200);
  //comp.calibrate();

  // Initial read using accelrometer at stand still
  ImuData imudata = imu.read();
  model.angle = imudata.angle;

  //also update initial trigonometric cache values
  model.sin_pitch = sinf(model.angle.pitch*PI/180);
  model.cos_pitch = cosf(model.angle.pitch*PI/180);
  model.sin_roll = sinf(model.angle.roll*PI/180);
  model.cos_roll = cosf(model.angle.roll*PI/180);
  model.tan_pitch = model.sin_pitch/model.cos_pitch;   

  delay(100);
  alt.setup();
  //delay(100);
  
  //warning: null division (gymbal lock)
  //yaw = comp.getHeadingCompensated(rollkalmanfilter.x_hat, pitchkalmanfilter.x_hat);
  ReceiverData data = receiver.recv();    // Lock until throttle is 0 first.
  while (data.ThrottleIn > 1050){
    data = receiver.recv();
  }

  
  
}
void loop() {
  last_active = micros();
  
  ImuData imudata = imu.read();

  model.predict(imudata.angle_rate);

  pitchkalmanfilter.bypass_predict();
  pitchkalmanfilter.correct(imudata.angle.pitch, model.angle.pitch);
  pitchkalmanfilter.update();

  rollkalmanfilter.bypass_predict();
  rollkalmanfilter.correct(imudata.angle.roll, model.angle.roll);
  rollkalmanfilter.update();

  model.coord_predict(imudata.accel);

  ReceiverData rd = receiver.recv();
  rd = receiver.to_anglemode(rd);       //IMPORTANT: forgetting this line will cause drone to fly away

  // calculate errors
  
  double pitch_error = rd.PitchIn - model.angle.pitch;
  double roll_error = rd.RollIn - model.angle.roll;

  double pitchrate_target, rollrate_target;

  // use square root curve to calculate desired rates
  if (pitch_error >= 0){
    pitchrate_target = pitch_error/(0.09*sqrt(pitch_error) + 0.27);
  } else {
    pitchrate_target = pitch_error/(0.09*sqrt(-pitch_error) + 0.27);
  }

  if (roll_error >= 0){
    rollrate_target = roll_error/(0.07*sqrt(roll_error) + 0.6);;
  } else {
    rollrate_target = roll_error/(0.07*sqrt(-roll_error) + 0.6);
  }
  

  double yawrate_target = 1.5*rd.YawIn;

  Angle euler_rate_target;
  euler_rate_target.pitch = pitchrate_target;
  euler_rate_target.roll = rollrate_target;
  euler_rate_target.yaw = yawrate_target;
  
  // convert to desired body rate

  Angle body_rate_target = model.euler_to_body(euler_rate_target);

  // feed the error to pid
  double pitchadjust = pitchratepid.calculate(body_rate_target.pitch - imudata.angle_rate.pitch);
  double rolladjust = rollratepid.calculate(body_rate_target.roll - imudata.angle_rate.roll);
  double yawadjust = yawratepid.calculate(body_rate_target.yaw - imudata.angle_rate.yaw);

  //Map to motor output

  float fl = rd.ThrottleIn + pitchadjust + rolladjust + yawadjust;
  float fr = rd.ThrottleIn + pitchadjust - rolladjust - yawadjust;
  float bl = rd.ThrottleIn - pitchadjust + rolladjust - yawadjust;
  float br = rd.ThrottleIn - pitchadjust - rolladjust + yawadjust;

  // Output to motor, lock until throttle is not 0
  // Danger: forgetting to convert rd.ThrottleIn to percentage will lead to flyaway

  if (rd.ThrottleIn > 0.01 && rd.ThrottleIn <= 1.0){
    motor.set_motor(fl, fr, bl, br);
  } else {
    motor.set_motor(0.0f, 0.0f, 0.0f, 0.0f);
    pitchratepid.reset();
    rollratepid.reset();
    yawratepid.reset();
    //alt.filter.reset();
  }

  // Telemetries
  // Height report
  if (micros() - last_altupdate > 260*1e3){  //height read at 4Hz

    //double h_barometer = alt.read();
    //report.h = h_barometer;
    /*
    double vertical_accel = (-imudata.accelX * model.sin_pitch + imudata.accelY * model.sin_roll * model.cos_pitch + imudata.accelZ * model.cos_roll * model.cos_pitch - 1) * 100 * 9.81 + 30;
    Serial.println(vertical_accel);
    vert_speed += 0.04 * vertical_accel;
    vert_speed = 0.998 * vert_speed + 0.002 * (h - h_barometer) / 0.04;

    double h_predicted = h - vert_speed * 0.04 - vertical_accel * 0.04 * 0.04 / 2.0;

    double past_h = h;

    h = h_alpha * h_predicted + (1 - h_alpha) * h_barometer;
    
    vert_speed = vert_speed * 0.992 + 0.008 * (h - past_h)/0.04;
    */
    h = alt.read();
    last_altupdate = micros();
  }

  //GPS report
  while (Serial4.available() > 0)
  {
    char gpsData = Serial4.read();

    // Send the read byte of data to the encode() function
    if (gps.encode(gpsData))
    {
        report.lat = gps.location.lat();
        report.lon = gps.location.lng();
    }
  }

    // actuate gimbal
  if (micros() - last_gimbal > 0.04*1e6){
    int compens = (int) model.angle.pitch;
    int gimbal_angle = (int)((rd.AuxChannel6In-1000)/2000 * 360);
    if (gimbal_angle < 10){
      gimbal_angle = 10;
    }
    if (compens > 70){
      compens = 70;
    }
    int final_angle = gimbal_angle + compens;
    if (final_angle < 10){
      final_angle = 10;
    }
    gimbal.write(final_angle);
    last_gimbal = micros();
  }

  // data gathering
  /*
    2B * 4 motors     = 8B
    2B * 3 gyro vals  = 6B
    2B * 3 accel vals = 6B 
    2B   alt          = 2B
    --------------------------
                        22B * 8/4ms << 115200 baud/s     
  */
  
  if (rd.AuxChannel5In > 1700) {
    Serial1.write(STARTBYTE);

    /*
    Serial1.write(motor.motor_report.fl);
    Serial1.write(motor.motor_report.fr);
    Serial1.write(motor.motor_report.bl);
    Serial1.write(motor.motor_report.br);

    Serial1.write(imu.raw_gyros.x);
    Serial1.write(imu.raw_gyros.y);
    Serial1.write(imu.raw_gyros.z);

    Serial1.write(imu.raw_accels.x);
    Serial1.write(imu.raw_accels.y);
    Serial1.write(imu.raw_accels.z);
    */
    Serial1.write((uint8_t*)&motor.motor_report, sizeof(RawMotor));
    Serial1.write((uint8_t*)&imu.raw_gyros, sizeof(RawImuData));
    Serial1.write((uint8_t*)&imu.raw_accels, sizeof(RawImuData));
    Serial1.write((uint8_t*)&alt.raw_alt, sizeof(uint16_t));

  }

  //FLight display report

  if (micros() - last_report > 80*1e3){     //write a report to the pi every 80ms = 12.5hz
    report.roll = model.angle.roll;
    report.pitch = model.angle.pitch;
    report.yaw = model.angle.yaw;
    report.bat = analogRead(A0)/1023.0*(50 + 4.7)/4.7;
    last_report = micros();
    report.h = h;
    report.lon = 0.0;
    report.lat = 0.0;

    //Serial1.write(STARTBYTE);
    //float checksum = report.roll + report.pitch + report.yaw + report.h;
    //Serial1.write((uint8_t*)&report, sizeof(report)); 
    //Serial1.write(checksum);  
    
    
    Serial.print("T: ");
    Serial.print(micros()-last_active);
    Serial.print(" H: ");
    Serial.println(h); 
    
    
    
  }

  while(micros() - last_active < DT*1000.0){}

}