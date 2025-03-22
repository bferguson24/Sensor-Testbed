// #include "motor.h"
#include <RoboClaw.h>
#include <SoftwareSerial.h>
#include "scoop.h"
#include "HX711.h"
#include "utility.h"



//Scoop Parameters
  #define address 0x80  
  const int xLimPin = 6;
  const int yLimPin = 7; 
  const int motor1_dir = 1;
  const int motor2_dir = 1; 

  const int Fx_dpin = 9;
  const int Fx_cpin = 10;
  const int Fy_dpin = 5;
  const int Fy_cpin = 6;
  int accelAddress = 0x53; // The ADXL345 sensor I2C address

//Pitch motor:
  int pitch_dir1 = 13;
  int pitch_dir2 = 12;
  int pitch_pwm = 2;
  int pitch_angle_pin = A0; 
  int pitch_angle_offset = 375; 
  PID_controller pitch_PID (5.0,0.0,1.0);

//Vibe Motor
  int vibe_dir1 = 10;
  int vibe_dir2 = 11;
  int vibe_pwm = 3;

//Load Cells
  HX711 Fx;
  HX711 Fy; 

  loadcell Fx_cell(&Fx, Fx_dpin, Fx_cpin);
  loadcell Fy_cell(&Fy, Fy_dpin, Fy_cpin); 

//Joint Limits:
  float xMax = 500; 
  float xMin = 10; 
  float yMax = -10; 
  float yMin = -80; 
  float pitchMax = 20; 
  float pitchMin = -20; 
  float vibeMin = 0;
  float vibeMax = 40; 

RoboClaw roboclaw(&Serial1, 1000);

scoop scoop(roboclaw, address, 
            xLimPin, yLimPin, 
            motor1_dir, motor2_dir, 
            pitch_dir1, vibe_dir1, 
            pitch_dir2, vibe_dir2, 
            pitch_pwm, vibe_pwm, 
            pitch_angle_pin, pitch_angle_offset, &pitch_PID, 
            Fx_cell, Fy_cell,
            accelAddress,
            xMax, xMin, yMax, yMin, pitchMax, pitchMin, vibeMin, vibeMax);



volatile bool timerFlag = false;  // Flag to indicate the interrupt triggered

void setup() {
  Serial.begin(115200);

  while (!Serial);
   for (int i = 0; i < 50; i++) {
    Serial.println();
  }

  // setupTimer2();  // Setup Timer2 
  // sei();  // Enable global interrupts
  scoop.init();

}



void loop() {
// Main Loop:


//Tasks
  scoop.serialTask(); // Reads incoming command from python and sends data back
  scoop.moveTask();  // Moves tool to commanded position if needed 
  scoop.readTask(); // get sensor data and store in data_out buffer 
  scoop.PIDtask();  // maintain PID setpoints 
  scoop.commandHandlerTask(); // Manage commands from python 



    // if (timerFlag) {
    // // Reset the flag to prevent multiple triggers
    // timerFlag = false;
    //   //Interrupt Functions
    //   scoop.readAllSensors();
    // }
}


// void setupTimer2() {
//   // Set Timer2 in CTC (Clear Timer on Compare Match) mode
//   // We want the interrupt to trigger at a specific frequency

//   // Assuming a 16 MHz clock and prescaler of 64 for a 1ms interrupt (this is more manageable)
//   // Frequency = (16MHz / 64) / 1000 = 250 Hz, or an interrupt every 1ms.
  
//   // Set the timer's prescaler to 64
//   TCCR2A = 0;  // Clear control register A
//   TCCR2B |= (1 << WGM12);  // CTC mode
//   TCCR2B |= (1 << CS22);   // Prescaler of 64

//   // Set the compare match register for 1ms interrupt (approximately)
//   OCR2A = 249;  // (16MHz / 64) / 1000 = 250 Hz, so OCR2A = 249 for a 1ms interval

//   // Enable interrupt on Compare Match A
//   TIMSK2 |= (1 << OCIE2A);
// }

// // Timer2 Compare Match A interrupt service routine
// ISR(TIMER2_COMPA_vect) {
//   // Set the flag to indicate the timer interrupt was triggered
//   timerFlag = true;
}








