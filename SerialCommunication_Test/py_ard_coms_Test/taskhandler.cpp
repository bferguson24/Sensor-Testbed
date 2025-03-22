#include "taskhandler.h"
#include "scoop.h"






// uint8_t buffer[] = [cmdbyte, xbytes[4], ybytes[4], pitchbytes[4], vibebytes[4]]




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
