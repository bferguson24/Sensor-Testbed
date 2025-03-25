#pragma once

#include <Arduino.h>
#include "scoop.h"

typedef uint8_t command_t;
enum {
  COMMAND_START,
  COMMAND_STOP,
  COMMAND_HOME,
  COMMAND_MOVE,
}; 

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
 
