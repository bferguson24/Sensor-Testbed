#include "sensors.h"

float loadcell::readForce(){
  Force = (this->m_loadcell->read() * Fgain) + Foffset;
  return Force;
}

