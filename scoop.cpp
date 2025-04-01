#include "scoop.h"
#include "utility.h"
#include <math.h>
#include <Arduino.h>
#include <Wire.h> 
#include "pid.h"
#include "taskhandler.h"

// Definitions
const float scoop::gearLead = 8.0f;
float scoop::encoder_output_ratio = 534.7 *1.0;
const float scoop::shaft_lead = 8.0f; // mm
const float grav = 9.81;


scoop* scoop::instance = nullptr;

//Scope resolution contructor
scoop::scoop(RoboClaw &roboclaw, uint8_t address, 
          Controller *controller,
          int xLimPin, int yLimPin, 
          int x_motor_dir, int y_motor_dir, 
          motor *pitchMotor, motor *vibeMotor,
          float xMax, float xMin, 
          float yMax, float yMin, 
          float pitchMax, float pitchMin, 
          float vibeMin, float vibeMax)

: roboclaw(roboclaw), address(address), 
  controller(controller), 
  xLimPin(xLimPin), yLimPin(yLimPin),
  motor1dir(motor1dir), motor2dir(motor2dir), 
  pitchMotor(pitchMotor), vibeMotor(vibeMotor), 
  xMax(xMax), xMin(xMin), 
  yMax(yMax), yMin(yMin), 
  pitchMax(pitchMax), pitchMin(pitchMin), 
  vibeMin(vibeMin), vibeMax(vibeMax)  
  {
  instance = this; 
}  


void scoop::init(){
  delay(1000);
  // Initialize all peripherals: 
  Serial.println("Scoop Starting");
  

  this->vibeMotor->init();
  this->pitchMotor->init(); 

  pinMode(this->xLimPin, INPUT_PULLUP);
  pinMode(this->yLimPin, INPUT_PULLUP);

  TCCR3A = (1 << WGM31);                 // Fast PWM mode 14
  TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31);  // Prescaler = 8
  ICR3 = 99;  // Set PWM period (for 20 kHz)


  // Attempt to connect to RoboClaw
  roboclaw.begin(38400);
  // if (millis() - startTime > roboclawTimeout) {
  //   Serial.println("\tERROR: RoboClaw connection timeout");
  //   return;  // Exit function or handle the error as needed
  // }
  Serial.println("\tRobo Claw Connected");
  
}

void scoop::set_command(command_t *command){
  this->activeCommand = *command; 
}

void scoop::process_command(uint8_t buffer[]){
  command_t *cmd =  (command_t*) buffer; 
  Serial.println(*cmd); 

  switch(*cmd){
    case COMMAND_IDLE:
      Serial.print("set cmd idle"); 
      instance->set_command(cmd); 
      break;

    case COMMAND_START: 
      Serial.print("set cmd start"); 
      instance->set_command(cmd); 
      break; 

    case COMMAND_STOP: 
      Serial.print("set cmd stop"); 
      instance->set_command(cmd); 
      break; 

    case COMMAND_HOME: 
      Serial.print("set cmd home"); 
      instance->set_command(cmd); 
      break; 
      
    case COMMAND_MOVE_MANUAL: 
      Serial.print("set cmd move manual"); 
      instance->set_command(cmd); 
      // Serial.print(instance->activeCommand); 
      break; 

    case COMMAND_MOVE_WAYPOINT: 
      Serial.print("set cmd move waypoint"); 
      instance->set_command(cmd); 
      
      move_cmd_t *move_cmd = (move_cmd_t*) (buffer);  
      instance->update_position(&(move_cmd->waypoint)); 
      break; 


    default:
      Serial.print("Unknown Command: ");
      Serial.println(*cmd);
      break;

  }
}

void scoop::move_waypoint(uint32_t vmax_x, uint32_t a_x, uint32_t vmax_y, uint32_t a_y){

  float x = current_waypoint->x;
  float y = current_waypoint->y;
  float pitch = current_waypoint->pitch;
  float vibe = current_waypoint->vibe_speed;
  int M1_dir = ((x > 0) ? 1 : -1) * scoop::motor1dir; 
  int M2_dir = ((y < 0) ? 1 : -1) * scoop::motor2dir; 

  //Motor 1 Command
  uint32_t M1_position = trans2rot(fabs(x)); 
  uint32_t M1_speed = trans2rot(vmax_x) * M1_dir;
  uint32_t M1_accel = trans2rot(a_x);

  //Motor 2 Command 
  uint32_t M2_position  = trans2rot(fabs(y)); 
  uint32_t M2_speed = trans2rot(vmax_y) * M2_dir;
  uint32_t M2_accel = trans2rot(a_y);

  Serial.print("WAYPOINT CALCULATION COMPLETE"); 

  // Serial.print("X = ");
  // Serial.print(x); 
  // Serial.print("Y = ");
  // Serial.print(y); 
  // Serial.print("Enc counts 1 = ");
  // Serial.print(M1_position);
  // Serial.print("Enc counts 2 = ");
  // Serial.print(M2_position); 

  // this->roboclaw.SpeedAccelDeccelPositionM1M2(address, M1_accel, M1_speed, M1_accel, M1_position, M2_accel, M2_speed, M2_accel, M2_position, 0);
  // this->vibeMotor->set_duty_cycle(vibe);
  // this->pitchMotor->set_angle(pitch); 

}

void scoop::move_task(){
  
  Serial.print("Active Command ");
  Serial.print(this->activeCommand); 
  switch(this->activeCommand){


    case COMMAND_IDLE:
      Serial.println("IDLE"); 
      this->roboclaw.SpeedM1M2(address, 0,0); 
    break;

    case COMMAND_START:
      Serial.println("START"); 
      this->roboclaw.SpeedM1M2(address, 0,0); 
    break;  

    case COMMAND_STOP: 
      Serial.println("STOP"); 
      this->roboclaw.SpeedM1M2(address, 0,0); 
      this->activeCommand = COMMAND_STOP; 
    break; 

    case COMMAND_HOME: 
      Serial.println("HOME"); 

      uint32_t speed = trans2rot(20); // [mm/s]
      uint32_t accel = trans2rot(1000); // [mm/s^2]
      switch (home_status){
        case STATE_SYSTEMS_OFF:
          //Vibe motor -> 0
          //Pitch motor ->
          home_status = STATE_WAITING_ENC1; 

          if (this->motor1dir == 1){
            this->roboclaw.BackwardM1(address, speed); 
          }
          if (this->motor1dir == -1){
            this->roboclaw.ForwardM1(address, speed);
          }
          home_status = STATE_WAITING_ENC1;     
        break; 


        case STATE_WAITING_ENC1:
          if (this->motor1dir == 1){
            this->roboclaw.BackwardM1(address, speed); 
          }
          if (this->motor1dir == -1){
            this->roboclaw.ForwardM1(address, speed);
          }

          if (digitalRead(this->xLimPin)){
            this->roboclaw.SpeedM1M2(address, 0,0); 
            this->M1_lim_counts = this->roboclaw.ReadEncM1(address); 
            STATE_MOVE_X0; 
          }
        break; 

        case STATE_MOVE_X0:
          float position = trans2rot(this->x0); 
          this->roboclaw.SpeedAccelDeccelPositionM1(address, accel, speed, accel, position,0);
        break;
      }
    break; 
        
    case 4:
      Serial.println("MOVE WAYPOINT"); 
      // this->move_waypoint(); 
    break; 
        
    case 5:
      Serial.println("MOVE MANUAL"); 

      // this->controller->multichannel_read();

      // waypoint_t waypoint_manual[] = {
      //   controller->a0.output,
      //   controller->a1.output,
      //   controller->a2.output,
      //   controller->a3.output
      // };
    
      // this->update_position(waypoint_manual);
      // this->move_waypoint(); 
    break; 

  }
}



void scoop::PID_task(){
//PITCH PID 
  this->pitchMotor->read_angle();
  float output = this->pitchMotor->pid->PID_task();
  this->pitchMotor->set_duty_cycle(output); 

//FUTURE VIBE MOTOR PID
}


//Scoop Functions

void scoop::displayspeed(void){
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4; 
  int32_t enc1 = scoop::roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t speed1 = scoop::roboclaw.ReadSpeedM1(address, &status2, &valid2);

  int32_t enc2 = scoop::roboclaw.ReadEncM2(address, &status3, &valid3);
  int32_t speed2 = scoop::roboclaw.ReadSpeedM2(address, &status4, &valid4);
  
  if(valid1){
    Serial.print("Encoder1:");
    Serial.print(enc1,DEC);
    Serial.print(" ");
    Serial.print(status1,HEX);
    Serial.print(" ");
  }
  if(valid2){
    Serial.print("Speed1:");
    Serial.print(speed1,DEC);
    Serial.print(" ");
  }

  if(valid3){
    Serial.print("Encoder2:");
    Serial.print(enc2,DEC);
    Serial.print(" ");
    Serial.print(status3,HEX);
    Serial.print(" ");
  }

  if(valid4){
    Serial.print("Speed2:");
    Serial.print(speed2,DEC);
    Serial.print(" ");
  }
  
  Serial.println();
}


void scoop::update_position(waypoint_t *waypoint){

  this->current_waypoint = waypoint; 
// Serial.print("X: ");
// Serial.print(this->current_waypoint->x, 6); 
// Serial.print("  ");

// Serial.print("Y: ");
// Serial.print(this->current_waypoint->y, 6);
// Serial.print("  ");

// Serial.print("Pitch: ");
// Serial.print(this->current_waypoint->pitch, 6);
// Serial.print("  ");

// Serial.print("Vibe Speed: ");
// Serial.println(this->current_waypoint->vibe_speed, 6);

}

void scoop::rampVibeSpeed(){
  float timeOffset = 0; 
  int command = 0;
  float loopTime = 0;

  for (int i = 0; i <= 6; i++) {
  // Set the duty cycle for the motor (increasing by 15 each iteration)
  timeOffset = millis(); 
  loopTime = 0;

  while(loopTime < 2000){
    int duty = command + (i * 10);
    this->vibeMotor->set_duty_cycle(duty);
    loopTime = millis() - timeOffset; 
    Serial.print(millis()/1000.0f, 4);
    Serial.print(",");
    readAccel();
    Serial.println();
    }
  }
}

uint32_t scoop::trans2rot(float value){
  uint32_t output = static_cast<uint32_t>((value / this->gearLead) * this->encoder_output_ratio);
  return output;
}
uint32_t scoop::rot2trans(float value){
  uint32_t output = static_cast<uint32_t>((value * this->gearLead) / this->encoder_output_ratio);
  return output;
}



