#pragma once

#include <Arduino.h>
#include "scoop.h"

typedef enum command {
  COMMAND_START,
  COMMAND_STOP,
  COMMAND_HOME,
  COMMAND_MOVE,
} command_t; 


typedef struct{
  float x;
  float y;
  float pitch; 
  float vibe_speed; 
} waypoint_t; 

typedef struct{
  command_t cmd;
  waypoint_t waypoint; 
} move_cmd_t;

