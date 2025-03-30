#include "motor.h"
#include "pid.h"
#include <Arduino.h>
#include "utility.h"

motor::motor(int pwmPin, int dir1Pin, int dir2Pin, int analogPin, float analog_offset, int invert_output, int invert_angle, PID_controller* PID = nullptr, sample* angle_sample = nullptr)
: pwm(pwmPin), dir1(dir1Pin), dir2(dir2Pin), analogReadPin(analogPin), analog_offset(analog_offset), invert_output(invert_output), invert_angle(invert_angle), pid(PID), angle_sample(angle_sample){}


void motor::setDutyCycle(float dutyCycle){
  int dir = (dutyCycle > 0) ? this->invert_output : -this->invert_output;
  int command = map(abs(dutyCycle), 0, 100, 0, 255); 

  //CW
  if (dir == 1){
    analogWrite(this->pwm, command);
    digitalWrite(this->dir1, 1);
    digitalWrite(this->dir2, 0);
  }
  //CCW
  else if (dir == -1){
    analogWrite(this->pwm, command);
    digitalWrite(this->dir1, 0);
    digitalWrite(this->dir2, 1);
  }
}


float motor::readAngle(){

  float rawVal = analogRead(this->analogReadPin); 
  float avgVal = this->angle_sample->moving_average(rawVal); 

  angle = this->invert_angle * (avgVal - this->analog_offset) * (360.0/1023.0); 


  return angle; 

}

float motor::set_angle(float angleSet){
  if (pid != nullptr) {
    
    float angleIn = readAngle(); 
    this->pid->processIn = angleIn;
    this->pid->setpoint = angleSet;
    this->angle = angleSet; 
    } 
  else {
    Serial.println("Null Pointer"); 
    }
    return angleSet;
}


void motor::init(){

  this->angle = 0; 
  this->speed = 0; 
  this->accel = 0; 
  this->decel = 0; 

  pinMode(this->pwm, OUTPUT);
  pinMode(this->dir1, OUTPUT);
  pinMode(this->dir2, OUTPUT);

}
