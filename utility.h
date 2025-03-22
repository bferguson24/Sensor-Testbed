#include "Arduino.h"

uint32_t myroundf(float value);

float clip(float value, float min, float max);

float analogStep(int analogPin, int deadBand, float stepSize, float min, float max, float *setPoint); 