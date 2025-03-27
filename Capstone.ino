// #include "motor.h"
#include <RoboClaw.h>
#include <SoftwareSerial.h>
#include "scoop.h"
#include "HX711.h"
#include "utility.h"
#include "taskhandler.h"
#include "packet.h"

//Packet 
SerialPacket packet; 


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
  float vibeMax = 40; 

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
  
  packet.setCallback(scoop::process_command);


  while (!Serial);
   for (int i = 0; i < 50; i++) {
    Serial.println();
  }

  // setupTimer2();  // Setup Timer2 
  // sei();  // Enable global interrupts
  scoop.init();

}



void loop() {
// Main Loop:

packet.read_state_task(); 

// scoop.update_position(&start_waypoint); 

}







