#pragma once

#include <Arduino.h>
#include "scoop.h"


// typedef __attribute__((packed))uint8_t command_t;
// enum {
//   COMMAND_START,
//   COMMAND_STOP,
//   COMMAND_HOME,
//   COMMAND_MOVE_WAYPOINT,
//   COMMAND_MOVE_MANUAL,
//   COMMAND_IDLE
// }; 

typedef enum {
  COMMAND_IDLE,
  COMMAND_START,
  COMMAND_STOP,
  COMMAND_HOME,
  COMMAND_MOVE_WAYPOINT,
  COMMAND_MOVE_MANUAL
} command_t;




typedef __attribute__((packed)) struct{
  float x;
  float y;
  float pitch; 
  float vibe_speed; 
} waypoint_t; 

typedef __attribute__((packed)) struct{
  command_t cmd;
  // float x;
  // float y;
  // float pitch; 
  // float vibe_speed; 
  waypoint_t waypoint; 
} move_cmd_t;
 