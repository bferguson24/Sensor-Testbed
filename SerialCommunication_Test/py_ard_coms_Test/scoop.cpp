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
      break;

    case COMMAND_STOP: 
      break; 
      
    case COMMAND_MOVE: 
      move_cmd_t *move_cmd = (move_cmd_t*) (buffer); 
      // scoop->move(move_cmd.waypoint);
      break; 
  }
}
