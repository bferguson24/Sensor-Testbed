#include "packet.h"
#include "Arduino.h"
#include "taskhandler.h"
#include "scoop.h"


SerialPacket::SerialPacket() {
    current_idx = 0; 
    incoming_packet_length = 0;
    sync_idx = 0;
    sync_found = false;
    
    setCallback(scoop::process_command);
    sync_word[0] = 0xDA;
    sync_word[1] = 0xBA;
    sync_word[2] = 0xD0;
    sync_word[3] = 0x00;
  };

void SerialPacket::setCallback(void (*cb)(uint8_t[])){
  callback = cb;
}



void SerialPacket::read_state_task(){
  if (Serial2.available() == 0){
    return;
  }
  uint8_t byte = Serial2.read(); 
  Serial.println(byte); 
  switch(state){

    case STATE_WAITING_SYNC_0:
      if(byte == sync_word[0]){
        state = STATE_WAITING_SYNC_1;
        break;
      }

    case STATE_WAITING_SYNC_1:
      if(byte == sync_word[1]){
        state = STATE_WAITING_SYNC_2;
        break;
      }
      else{
        state = STATE_WAITING_SYNC_0;
        break; 
      }

    case STATE_WAITING_SYNC_2:
      if(byte == sync_word[2]){
        state = STATE_WAITING_SYNC_3;
        break;
      }
      else{
        state = STATE_WAITING_SYNC_0;
        break; 
      }

    case STATE_WAITING_SYNC_3:
      if(byte == sync_word[3]){
        state = STATE_WAITING_LEN;
        // Serial.println("Sync Word Found"); 
        break;
      }
      else{
        state = STATE_WAITING_SYNC_0;
        break; 
      }

    case STATE_WAITING_LEN:
      this->incoming_packet_length = byte; 
      // Serial.print("Incoming Length = "); 
      // Serial.println(incoming_packet_length); 
      state = STATE_WAITING_READ_BYTES; 
      break; 

    case STATE_WAITING_READ_BYTES:
      if (this->current_idx < this->incoming_packet_length - 1){
        this->buffer[this->current_idx] = byte; 
        // Serial.println(buffer[current_idx]); 
        current_idx++;
        
      } else {
        Serial.println("data read complete");
        callback(this->buffer);

        this->current_idx = 0; 
        state = STATE_WAITING_SYNC_0; 
      }
      break; 

  }
}
