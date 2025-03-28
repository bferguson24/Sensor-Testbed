#pragma once

#include <RoboClaw.h>
#include "pid.h"
#include "taskhandler.h"

//Peyton 
//include any accel libraries here: 



typedef enum{
  STATE_SYSTEMS_OFF,
  STATE_WAITING_ENC1,
  STATE_MOVE_X0,
  STATE_WAITING_ENC2,
  STATE_MOVE_Y0,
  STATE_MOVE_YPRE, 
  STATE_MEASURE_YGND,
  STATE_WAITING_PAUSE1,
  STATE_RETURN_Y0,
  STATE_HOME_COMPLETE
} homing_state_t; 

class scoop {

private:
//Pinout
  RoboClaw &roboclaw;
  uint8_t address;
  int pitchPWMpin;
  int pitchDirPin; 
  int xLimPin;
  int yLimPin;
  int motor1dir;
  int motor2dir;

//Joint Limits:
  float xMax;
  float xMin;
  float yMax;
  float yMin;
  float pitchMax;
  float pitchMin; 
  float vibeMin;
  float vibeMax; 

//Homing Parameters
  uint32_t M1_lim_counts; 
  uint32_t M2_lim_counts;
  float y_gnd;  

//Constants
  float x0;
  float y0; 
  float y_gnd_pre; 
  static const float gearLead; // NA
  static float encoder_output_ratio; // NA
  static const float shaft_lead; // mm 

//Control
  command_t activeCommand; 
  homing_state_t home_status; 
  static scoop* instance; 

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
    float set_angle(float angleInput);
    void setSpeed(float freq); 
    void setDutyCycle(float dutyCycle); 

  private: 
  int analogReadPin; 
  float analog_offset; 
  }; 

public:

    waypoint_t *current_waypoint; 
    motor pitchMotor; 
    motor vibeMotor; 

    //Contructor
    scoop(RoboClaw &roboclaw, uint8_t address, 
          int xLimPin, int yLimPin, 
          int motor1dir, int motor2dir,
          int pitch_dir1, int vibe_dir1, 
          int pitch_dir2, int vibe_dir2,
          int pitch_pwm, int vibe_pwm,
          int pitch_angle_pin, int pitch_offset, 
          PID_controller *pitch_pid,
          float xMax, float xMin, 
          float yMax, float yMin, 
          float pitchMax, float pitchMin, 
          float vibeMin, float vibeMax);


    
    void move_task(uint32_t vmax_x = 10000, uint32_t a_x = 10000 , uint32_t vmax_y = 10000, uint32_t a_y = 10000); 
    void PID_task();
    void commandHandlerTask(); 
    static void process_command(uint8_t *buffer); 
    void update_position(waypoint_t *waypoint);
    void set_command(command_t *command); 
    void home();
    void excavateSequence(); 
    // void moveAbsolute(float x, float y, float pitch, float vibe = 0, uint32_t vmax_x = 10000, uint32_t a_x = 10000 , uint32_t vmax_y = 10000, uint32_t a_y = 10000); //Move to absolute position using Home Values
    // bool moveRelative(double x, double y, double pitch, uint32_t vmax_x = 10000, uint32_t a_x = 10000 , uint32_t vmax_y = 10000, uint32_t a_y = 10000); // Move from current position by distance x y pitch 
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


