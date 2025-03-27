#include "packet.h"
#include "taskhandler.h"

SerialPacket packet; 
scoop scoop; 
// SensorSet sensors; 

// uint8_t buffer[512]; 
// // int current_idx = 0; 

// float x; 



// uint8_t * x_bytes = (uint8_t *) &x;    // Treat x as pointer to uint8_t to address of x 
// memcpy(buffer, x_bytes, 4); 

// x = 100.00 

// uint8_t buffer = {0xDA, 0x00, 0x00, 0x00, .... 18 bytes}
// int xOffset = (sync_idx + N); 

// float xinput;

// typedef struct {
//   command_t cmd;
//   waypoint_t waypoint;
// } move_cmd_t; 

// typdef struct {
//   float x;
//   float y;
//   float z;
// } waypoint_t


// void packet::ReadTask(){

//   buffer = ... 

//   processCommand(buffer);
// }

// void scoop::home(){
//   scoop.isHoming = True; 
// }

// void scoop::task(){
//   //Big state machine that manages whether homing status = true or if move command; 

// }
// //Runs Quickly, no blocking


//Keep in mind that MCU has 3-4 fifo buffer, so you can't read 
// void processCommand(uint8_t *buffer){

//   command_t cmd = (command_t)buffer.cmd; 

//   switch(cmd){

//     case START:

//     case STOP:

//     case HOME:
//       scoop.home(); 

//     case MOVE:

//   }  
// }


//Task, always executed every single time, doesn't require any params 



// move_cmd_t* command = (move_cmd_t*) buffer

// //This isn't  
// waypoint_t* waypoint = &(command->waypoint);  //Equal (More conventional)
// waypoint_t* waypoint = &( (*command).waypoint);  //Equal

// //This is correct
// waypoint_t waypoint = command->waypoint;  //Equal
// waypoint_t waypoint = *command.waypoint;  //Equal



// IK(*command.x); 
// IK(*((move_cmd_t *) buffer).x,*((move_cmd_t *) buffer).y,*((move_cmd_t *) buffer).z);


// memcpy(xinput, (buffer + Xoffset), 4); 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Print Statements
  Serial2.begin(115200); // Incoming python commands
  packet.setCallback(scoop::process_command);
}


void loop() {

packet.read_state_task(); 
// sensors.write_task(); 


  // packet.read_task();
  // if (Serial2.available() > 0){
  
  //   buffer[current_idx] = Serial2.read();
  //   current_idx++;

  //    // Check if we need to reset the index
  //   if (current_idx >= sizeof(buffer)) {
  //     current_idx = 0;  // Reset index if it exceeds buffer size
  //   }

  //   Serial.print("Data received. Current index: ");
  //   Serial.println(buffer[current_idx - 1]);
  // }
}
