#pragma once

#include <stdint.h> 


typedef enum {
  COMMAND_HOME,
  COMMAND_MOVE,
  COMMAND_START,
  COMMAND_STOP,
} command_t;


__attribute__((packed))
typedef struct{
  command_t cmd;
  float x;
  float y; 
  float pitch;
} move_cmd_t;

typedef struct {
  command_t cmd;
} home_cmd_t; 


typedef struct {
  command_t cmd;
} start_cmd_t; 

typedef struct {
  command_t cmd;
} stop_cmd_t;

tpyedef struct {
  float x;
  float y;
  float z;
} wayPoint_t

typedef struct{
  float ax;
  float ay;
  float az;
  float Fx;
  float Fy; 
} data_out_t

