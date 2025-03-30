//General PID Functions
#pragma once

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
  float processMin = 0;
  float processMax = 0;
  float deadband = 0.0f;
  float minValue = 3.0;
  bool useSqrtErrorP = false;
  float lastError = 0.0f; 

  //Constructor:
  PID_controller(float Kp = 0.0f, float Ki = 0.0f, float Kd = 0.0f,
                 float processMin = 0.0f, float processMax = 0.0f, 
                 float deadband = 0.0f, float minValue = 0.0f)

  : kp(Kp), ki(Ki), kd(Kd), processMin(processMin), processMax(processMax),
    deadband(deadband), minValue(minValue), useSqrtErrorP(useSqrtErrorP), 
    lastError(0.0f) {}

  float PID_task();
  void PID_init(); 
};


