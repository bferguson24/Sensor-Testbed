#pragma once
#include <stdint.h>


class scoop{
  public:

  scoop(); 
    void move(float x, float y, float pitch);
    void home();
    void stop();
    void resume(); 
    void process_command(uint8_t *buffer); 
};