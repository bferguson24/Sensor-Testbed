#pragma once

#include "HX711.h"

class loadcell{
  public:
    float Force;
    float Foffset;
    float Fgain; 

    float readForce();
    void tare();

  //Constructor
    loadcell(HX711 *load_cell, int dataPin, int clockPin)
      : dataPin(dataPin), clockPin(clockPin) {
      m_loadcell = load_cell;
      Force = 0.0f;
      Fgain = 1.0;
      Foffset = 0.0;
    }

  private: 
    HX711 *m_loadcell;
    int dataPin;
    int clockPin;
}; 