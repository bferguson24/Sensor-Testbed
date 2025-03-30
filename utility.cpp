#include "stdint.h"
#include "utility.h"


uint32_t myroundf(float value) {
    return (value >= 0) ? (uint32_t)(value + 0.5f) : (uint32_t)(value - 0.5f);  
}

float clip(float value, float min, float max){ 
 if (value < min) return min; 
 if (value > max) return max; 
 return value;  
}

float analogStep(int analogPin, int deadBand, float stepSize, float min, float max, float *setPoint){
  float avgAnalog = 1023.0/2.0;
  int rawVal = analogRead(analogPin);
  float val = 0.0; 

  if (rawVal > avgAnalog + deadBand){
    val = (rawVal - (avgAnalog + deadBand)) / (1023.0 - (avgAnalog + deadBand)) * stepSize;
  }
  else if (rawVal < (avgAnalog - deadBand)){
    val = (rawVal - avgAnalog) / (avgAnalog - 0.0) * stepSize;
  }

  *setPoint += val; 
  *setPoint = clip(*setPoint, min, max);
  
  return *setPoint; 
}


sample::sample(float alpha, float initial_avg)
  :alpha(alpha), average(initial_avg){}

float sample::moving_average(float newVal){
  average = (alpha * newVal) + (1- alpha) * average;
  return average;   
}
