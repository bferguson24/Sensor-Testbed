#include "packet.h"
#include "Arduino.h"
#include "taskhandler.h"
#include "scoop.h"


SerialPacket::SerialPacket() {
    current_idx = 0; 
    incoming_packet_length = 0;
    sync_idx = 0;
    sync_found = false;

    sync_word[0] = 0xDA;
    sync_word[1] = 0xBA;
    sync_word[2] = 0xD0;
    sync_word[3] = 0x00;
  };

void SerialPacket::setCallback(void (*cb)(uint8_t[])){
  callback = cb;
}



// typedef enum{
//   STATE_WAITING_SYNC_0,
//   STATE_WAITING_SYNC_1,
//   STATE_WAITING_SYNC_2,
//   STATE_WAITING_SYNC_3,
//   STATE_WAITING_LEN,
//   STATE_WAITING_CMD,
//   STATE_WAITING_READ_BYTES,
// } state_t; 

void SerialPacket::read_state_task(){
  if (Serial2.available() == 0){
    return;
  }
  uint8_t byte = Serial2.read(); 

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
        // Serial.println("data read complete");
        callback(this->buffer);

        this->current_idx = 0; 
        state = STATE_WAITING_SYNC_0; 
      }
      break; 

  }
}


void SerialPacket::read_task() { 
  if (Serial2.available() == 0){
    return;
  }

  if(sync_found){

    // Read a byte into the buffer
    Serial.print("Byte index = ");
    Serial.print(current_idx); 
    Serial.print("  "); 

    
  //Conventially read all available bytes into buffer, but that would fuck shit up 

    buffer[current_idx] = Serial2.read(); 

    Serial.print("Value = ");
    Serial.print(buffer[current_idx], HEX); 
    Serial.print("  "); 

    if (current_idx >= 3){
    // Serial.print("current index > 3");
    // Serial.print("  "); 
      bool found_sync = true; 
      for (int i = 0; i < 4 ; i++){
        // Serial.println(); 
        // Serial.print(   "Sync Byte = ");
        // Serial.print(sync_word[3 - i], HEX);
        // Serial.print("  "); 
        // Serial.print(   "Index Byte = ");
        // Serial.print(buffer[current_idx - i], HEX);

        if (buffer[current_idx - i] != sync_word[3 - i]){
          found_sync = false; 
          Serial.print("sync_found = false"); 
          Serial.print("  "); 
          Serial.print(current_idx); 
          Serial.print("  ");
          Serial.print(buffer[current_idx - i]);
          Serial.print("  "); 
          Serial.print(sync_word[3-i]); 
          Serial.println(); 
          break;
          }      
      }

      if (found_sync) {
        sync_found = true; 
        // If sync word is found, calculate the sync index
        sync_idx = current_idx - 3; 
        Serial.print("Sync  index = ");
        Serial.println(sync_idx);
      }

      Serial.println();

        // Only update incoming_packet_length when the byte after the sync word is read
      if (sync_found && current_idx == (sync_idx + 4)) {
        incoming_packet_length = buffer[current_idx];
        // Serial.print("Incoming packet length = ");
        // Serial.println(incoming_packet_length);
      }

    
      if (current_idx >= sizeof(buffer)){
        current_idx = 0; 
      }
    }

    current_idx++; 
  } 

  Serial.print("Current index = ");
  Serial.print(current_idx); 

  Serial.print("  ");

  Serial.print("sync index = ");
  Serial.print(sync_idx);
  Serial.print("  ");

  Serial.print("sync status= ");
  Serial.print(sync_found);
  Serial.print("  ");


  Serial.print("Incoming length = ");
  Serial.print(incoming_packet_length); 
  Serial.print("  ");

  Serial.println(); 
} 
