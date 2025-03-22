//General PID Functions
#pragma once

#include  "Arduino.h"


class PID_controller {
public:
  float setpoint = 0.0f; 
  volatile float processIn = 0.0f;
  float error = 0.0f; 
  float ierror = 0.0f; 
  float derror = 0.0f; 
  float kp = 0.0f; 
  float ki = 0.0f; 
  float kd = 0.0f; 
  float processMin = -100;
  float processMax = 100;
  float deadband = 0.0f;
  float minValue = 0.0f;
  bool useSqrtErrorP = false;
  float lastError = 0.0f; 

  float PID_task(float processIn);
  void PID_init(); 

  //Constructor:
  PID_controller(float Kp, float Ki, float Kd)
    :kp(Kp), ki(Ki), kd(Kd) {
  }
};

uint32_t myroundf(float value);

float clip(float value, float min, float max);
float mapf(float x, float in_min, float in_max, float out_min, float out_max); 

