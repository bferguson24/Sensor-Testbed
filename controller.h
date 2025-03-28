#pragma once
#include <stdint.h>
#include <Arduino.h> 

typedef struct{
  float output; 
  int analog_pin; 
  int deadband; 
  float stepsize;
  float min;
  float max; 
}adc_channel_t; 



class Controller{
  public:

  adc_channel_t a0;
  adc_channel_t a1;
  adc_channel_t a2;
  adc_channel_t a3;

  Controller(float deadband, float stepsize); 
  
  void analog_step(adc_channel_t *channel); 
  void multichannel_read();

  private: 
    float stepsize;
    float deadzone; 
};