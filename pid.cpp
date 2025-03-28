#include "pid.h"
#include "utility.h"
#include "math.h"

float PID_controller::PID_task() { 
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
