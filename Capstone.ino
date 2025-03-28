// #include "motor.h"
#include <RoboClaw.h>
#include <SoftwareSerial.h>
#include "scoop.h"
#include "HX711.h"
#include "utility.h"
#include "taskhandler.h"
#include "packet.h"
#include "controller.h"

//Packet 
SerialPacket packet; 
Controller controller(30, 0.5); 


//Scoop Parameters
  #define address 0x80  
  const int xLimPin = 6;
  const int yLimPin = 7; 
  const int motor1_dir = 1;
  const int motor2_dir = 1; 

//Pitch motor:
  int pitch_dir1 = 13;
  int pitch_dir2 = 12;
  int pitch_pwm = 2;
  int pitch_angle_pin = A0; 
  int pitch_angle_offset = 375; 
  PID_controller pitch_PID (5.0,0.0,1.0);

//Vibe Motor
  int vibe_dir1 = 10;
  int vibe_dir2 = 11;
  int vibe_pwm = 3;

//Joint Limits:
  float xMax = 500; 
  float xMin = 10; 
  float yMax = -10; 
  float yMin = -80; 
  float pitchMax = 20; 
  float pitchMin = -20; 
  float vibeMin = 0;
  float vibeMax = 100; 

RoboClaw roboclaw(&Serial1, 1000);

scoop scoop(roboclaw, address, 
            xLimPin, yLimPin, 
            motor1_dir, motor2_dir, 
            pitch_dir1, vibe_dir1, 
            pitch_dir2, vibe_dir2, 
            pitch_pwm, vibe_pwm, 
            pitch_angle_pin, pitch_angle_offset, &pitch_PID, 
            xMax, xMin, yMax, yMin, pitchMax, pitchMin, vibeMin, vibeMax);



waypoint_t start_waypoint = {1,2,3,4};

void setup() {
  Serial.begin(115200); // Print Statements
  Serial2.begin(115200); // Incoming python commands

  pinMode(3, OUTPUT); 
  pinMode(2, OUTPUT); 

  packet.setCallback(scoop::process_command);

  while (!Serial);
   for (int i = 0; i < 50; i++) {
    Serial.println();
  }
  scoop.init();
}


void loop() {

controller.multichannel_read();

uint32_t y = controller.a0.output; 
uint32_t x = controller.a1.output; 
float vibe = controller.a2.output; 

// roboclaw.SpeedAccelDeccelPositionM1M2(address, accel, speed, accel, x, accel, speed, accel, y, 0);
scoop.vibeMotor.setDutyCycle(vibe); 
}




// scoop.move_task(); 


// Serial.print(0.0); 
// Serial.print(","); 
// Serial.print(controller.a0.output); 
// Serial.print(","); 
// Serial.print(controller.a1.output); 
// Serial.print(",");
// Serial.println(50.0); 









