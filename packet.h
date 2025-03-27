#pragma once

#include <stdint.h>
#include <Arduino.h> 
#include "scoop.h"

typedef enum{
  STATE_WAITING_SYNC_0,
  STATE_WAITING_SYNC_1,
  STATE_WAITING_SYNC_2,
  STATE_WAITING_SYNC_3,
  STATE_WAITING_LEN,
  STATE_WAITING_READ_BYTES,
  STATE_DATA_COMPLETE,
} state_t; 

class SerialPacket {
  public:
    // using CommandCallback = void (Scoop::*)(uint8_t*); 
    // Scoop *scoopInstance = nullptr;

    // CommandCallback callback = nullptr;
    SerialPacket();

    void read_task();
    void read_state_task(); 

   
    void (*callback)(uint8_t[]); 

    void setCallback(void (*cb)(uint8_t[]));



  private: 

    state_t state; //Current State of Process
    uint8_t buffer[512]; // Store incoming data
    uint8_t current_idx; // Current index/position
    int incoming_packet_length; // Length of packet currently being read
    uint8_t sync_idx; // Location of sync word in current index
    bool sync_found; // Check if sync word is found or not
    uint8_t sync_word[4]; // sync word
};

