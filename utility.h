#include "Arduino.h"
#pragma once 

uint32_t myroundf(float value);

float clip(float value, float min, float max);

float analogStep(int analogPin, int deadBand, float stepSize, float min, float max, float *setPoint); 



class sample{
  public:

  sample(float alpha, float initial_avg);

    float average;
    float alpha; 
    float moving_average(float newVal);
}; 

