#pragma once

#include <RoboClaw.h>
#include "HX711.h"
#include "pid.h"
#include "sensors.h"

//Peyton 
//include any accel libraries here: 


class scoop {

private:
  //Private Variables
  RoboClaw &roboclaw;
  uint8_t address;
  int pitchPWMpin;
  int pitchDirPin; 

//Used to correct Direction of motors;
  int motor1dir;
  int motor2dir;
  
  int xLimPin;
  int yLimPin;
  int Fx_DT;
  int Fx_SCK;
  int Fy_DT;
  int Fy_SCK;
  int accelAddress; 

//Communication Stuff;
  bool SYNC_STATUS; 
  uint8_t incomingLength; 


  class motor {
  public: 
    int angle; 
    int speed;
    int accel;
    int decel; 
    int dir1; 
    int dir2;
    int pwm; 

    PID_controller *pid; 

    motor(int pwmPin = 0, int dir1Pin = 0, int dir2Pin = 0, int analogPin = 0, float analog_offset = 0.0, PID_controller* PID = nullptr)
    : pwm(pwmPin), dir1(dir1Pin), dir2(dir2Pin), analogReadPin(analogPin), analog_offset(analog_offset), pid(PID){}

    float readAngle();
    float setAngle(float angleInput);
    void setSpeed(float freq); 
    void setDutyCycle(float dutyCycle); 

  private: 
  int analogReadPin; 
  float analog_offset; 
  }; 

public:
    //Current Position Command
    float x;
    float y; 
    float pitch; 
    float vibe; 

    //Accelerometer Data
    float ax;
    float ay;
    float az;
  
    //Joint Limits:
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float pitchMax;
    float pitchMin; 
    float vibeMin;
    float vibeMax; 

    //These parameters shouldn't be floats or ints, not sure on exact size?

    //HOME DATA
    int homeXenc;
    int homeYenc; 
    float groundDistance; 

    // System Parameters
    static const float gearLead; // NA
    static float encoder_output_ratio; // NA
    static const float shaft_lead; // mm 


    motor pitchMotor; 
    motor vibeMotor; 

    loadcell &Fx; 
    loadcell &Fy; 

    //Contructor
    scoop(RoboClaw &roboclaw, uint8_t address, 
          int xLimPin, int yLimPin, 
          int motor1dir, int motor2dir,
          int pitch_dir1, int vibe_dir1, 
          int pitch_dir2, int vibe_dir2,
          int pitch_pwm, int vibe_pwm,
          int pitch_angle_pin, int pitch_offset, 
          PID_controller *pitch_pid,
          loadcell &FX_cell, loadcell &FY_cell,
          // int Fx_DT, int Fx_SCK, int Fy_DT, int Fy_SCK, 
          int I2cAdd,
          float xMax, float xMin, 
          float yMax, float yMin, 
          float pitchMax, float pitchMin, 
          float vibeMin, float vibeMax);


    
    void serialTask();
    void moveTask(); 
    void readTask(); 
    void PIDtask(); 
    void commandHandlerTask(); 


    void home();
    void excavateSequence(); 
    void moveAbsolute(float x, float y, float pitch, float vibe = 0, uint32_t vmax_x = 10000, uint32_t a_x = 10000 , uint32_t vmax_y = 10000, uint32_t a_y = 10000); //Move to absolute position using Home Values
    bool moveRelative(double x, double y, double pitch, uint32_t vmax_x = 10000, uint32_t a_x = 10000 , uint32_t vmax_y = 10000, uint32_t a_y = 10000); // Move from current position by distance x y pitch 
    void groundLevel();
    void readAccel();
    void readAllSensors();
    void displayspeed();
    void inputControl(int analogX, int analogY, int analogPitch, int analogVibe, float deadBand = 30, float stepSize = 0.5);
    void init(); 
    void rampVibeSpeed(); 
    uint32_t trans2rot(float value);
    uint32_t rot2trans(float value); 
};


