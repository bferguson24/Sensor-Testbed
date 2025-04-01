#pragma once
#include "pid.h"  
#include "utility.h"  


  class motor {
  public: 
    float angle; 
    float speed;
    float accel;
    float decel; 

    PID_controller *pid; 
    sample* angle_sample;

    motor(int pwmPin = 0, int dir1Pin = 0, 
          int dir2Pin = 0, int analogPin = 0, 
          float analog_offset = 0.0, 
          int invert_output = 1, int invert_angle = 1, 
          PID_controller* PID = nullptr, 
          sample* angle_sample = nullptr
        ); 

    float read_angle();
    float set_angle(float angleInput);
    void setSpeed(float freq); 
    void set_duty_cycle(float dutyCycle); 
    void init(); 

  private: 
  int analogReadPin; 
  float analog_offset; 
  int invert_angle; 
  int invert_output; 
  int dir1; 
  int dir2;
  int pwm; 
  }; 