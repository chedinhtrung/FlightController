#ifndef DATASTRUCT
#define DATASTRUCT

struct Angle {
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
};

struct Coords {
    float z = 0.0;
    float x = 0.0;
    float y = 0.0;
};

struct Report {
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
  float lon = 0.0;
  float lat = 0.0;
  float h = 0.0;
  float bat = 0.0; 
};

#endif