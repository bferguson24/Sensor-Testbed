#include "pid.h"
#include "math.h"
#include "stdint.h"
#include  "Arduino.h"

float PID_controller::PID_task(float processIn) { 
    this->processIn = processIn;  // Keeping 'this->' because it's storing the input
    
    float error = setpoint - processIn;  

    // **Process Control Deadband**
    if (error > deadband)
        error -= deadband; 
    else if (error < -deadband)
        error += deadband; 
    else
        error = 0;
    
    // **Square Root Error Correction**
    if (useSqrtErrorP) {
        if (error < 0)
            error = -sqrt(fabs(error));
        else 
            error = sqrt(fabs(error));
    }

    this->error = error;  // Keeping 'this->' because it updates the class member

    if (ki != 0.0f) {
        ierror += error;
        ierror = clip(ierror, processMin / ki, processMax / ki);  
    } else {
        ierror = 0;  
    }

    derror = error - lastError;

    float output = (kp * error) + (ki * ierror) + (kd * derror);
    
    if (output != 0)
        output = (output > 0) ? output + minValue : output - minValue;  

    output = clip(output, processMin, processMax);  

    lastError = error;

    return output;  
}

uint32_t myroundf(float value) {
    return (value >= 0) ? (uint32_t)(value + 0.5f) : (uint32_t)(value - 0.5f);  
}

float clip(float value, float min, float max){ 
 if (value < min) return min; 
 if (value > max) return max; 
 return value;  
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
