#include "scoop.h"
#include "taskhandler.h"   



scoop::scoop(){}

void move(float x, float y, float pitch){

}

void home(){
}

void stop(){
}

void resume(){
}


void scoop::process_command(uint8_t buffer[]){
  command_t *cmd =  (command_t*) buffer; 

  switch(*cmd){
    case COMMAND_START:
      Serial.println("START"); 

      break;

    case COMMAND_STOP: 
      Serial.println("STOP"); 
      break; 

    case COMMAND_HOME: 
      Serial.println("HOME COMMAND"); 
      break; 

      
    case COMMAND_MOVE: 
      Serial.print("MOVE COMMAND"); 
      Serial.print("  ");
      
      move_cmd_t *move_cmd = (move_cmd_t*) (buffer);  
      Serial.print("X: ");
      Serial.print(move_cmd->waypoint.x, 6); 
      Serial.print("  ");

      Serial.print("Y: ");
      Serial.print(move_cmd->waypoint.y, 6);
      Serial.print("  ");

      Serial.print("Pitch: ");
      Serial.print(move_cmd->waypoint.pitch, 6);
      Serial.print("  ");

      Serial.print("Vibe Speed: ");
      Serial.println(move_cmd->waypoint.vibe_speed, 6);
      break; 
  }
}
