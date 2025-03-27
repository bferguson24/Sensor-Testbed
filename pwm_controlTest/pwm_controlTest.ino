#include "pid.h"

int enApin = 3;
int in1 = 12;
int in2 = 13;
float angle = 0;

int encPin = A1;
int angleOffset = 388;
PID_controller pid(5,0,0); 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(enApin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  TCCR3A = (1 << WGM31);                 // Fast PWM mode 14
  TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS31);  // Prescaler = 8
  ICR3 = 99;  // Set PWM period (for 20 kHz)
}

void setDutyCycle(int dutyCycle, int Apin, int in1, int in2){
  int dir = (dutyCycle > 0) ? 1 : -1;
  int command = map(abs(dutyCycle), 0, 100, 0, 255); 

  //CW
  if (dir == 1){
    analogWrite(Apin, command);
    digitalWrite(in1, 1);
    digitalWrite(in2, 0);
  }

  else if (dir == -1){
    analogWrite(Apin, command);
    digitalWrite(in1, 0);
    digitalWrite(in2, 1);
  }
}

float readAngle(){
  int rawVal = analogRead(encPin);
  float angle = ((rawVal - angleOffset) * 360.0f/1023.0f); 
  return angle; 
}


float incrementAngle(int analogPin, float &currentAngle, float min, float max) {  // Pass by reference
  // Read Raw value;
  int deadBand = 30;  // Initialize deadBand to avoid undefined behavior
  int stepSize = 1; 
  int maxAnalog = 1023; 
  int meanAnalog = (maxAnalog/2);


  float val = maxAnalog - analogRead(analogPin);  // Read the analog input as an integer
  val = (float)val;  // Convert to float

  if (val > (meanAnalog + deadBand)) {
    val -= deadBand;
    val = mapf(val, meanAnalog + deadBand, maxAnalog, 0, stepSize);  // Use mapf for float mapping
  } 
  else if (val < (meanAnalog - deadBand)) {
    val += deadBand;
    val = mapf(val, meanAnalog - deadBand, 0, 0, -stepSize);  // Use mapf for float mapping
  } 
  else {
    val = 0;  // Inside deadband
  }

  float outputAngle = val + currentAngle; 
  outputAngle = clip(outputAngle, min, max);
  currentAngle = outputAngle; // This now modifies the original variable
  return outputAngle;   
}


void setAngle(float angle, int d1, int d2, int pwm){

//PROCESS
  float angleIn = readAngle(); //processIn 
  pid.processIn = angleIn;

  pid.setpoint = angle; //Setpoint
  float output = pid.PID_task(angleIn);
  setDutyCycle(output, pwm, d1, d2); 

//PRINT LINES
  // Serial.print("  Angle Set = "); //Commanded Angle
  Serial.print(angle);
  Serial.print(",");

  // Serial.print("  Angle Measured = "); //Measured Angle
  Serial.print(angleIn); 
  Serial.print(","); 

  // Serial.print("  dutyCycle Output = ");
  Serial.print(output);
  Serial.println();
}

void loop() {

float command = incrementAngle(A0, angle, -100,100); 
Serial.println(command); 
setDutyCycle(command, enApin, in1, in2);


// float angleVal = incrementAngle(A0, angle, -45, 45); 
// // int command = map(analogRead(A0),0,1023,-45,45);
// setAngle(angleVal, in1, in2, enApin); 
// readAngle(); 
}
