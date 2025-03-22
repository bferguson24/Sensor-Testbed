volatile bool timerFlag = false;  // Flag to indicate the interrupt triggered

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  setupTimer2();  // Setup Timer2 instead of Timer1

  sei();  // Enable global interrupts

  delay(1000);
}

void loop() {
  Serial.print("Timer Counter: ");
  Serial.println(TCNT2);  // Show the current value of Timer2 counter

  // Check if the Compare Match Flag is set
  if (TIFR2 & (1 << OCF2A)) {
    Serial.println("Compare Match Flag Set!"); // Debug: Interrupt should have occurred
  }

  if (timerFlag) {
    // Reset the flag to prevent multiple triggers
    timerFlag = false;

    // Your logic when the timer interrupt triggers
    Serial.println("Timer Interrupt Triggered!");
  }

  // Other code for the main loop goes here
}

void setupTimer2() {
  // Set Timer2 in CTC (Clear Timer on Compare Match) mode
  // We want the interrupt to trigger at a specific frequency

  // Assuming a 16 MHz clock and prescaler of 64 for a 1ms interrupt (this is more manageable)
  // Frequency = (16MHz / 64) / 1000 = 250 Hz, or an interrupt every 1ms.
  
  // Set the timer's prescaler to 64
  TCCR2A = 0;  // Clear control register A
  TCCR2B |= (1 << WGM12);  // CTC mode
  TCCR2B |= (1 << CS22);   // Prescaler of 64

  // Set the compare match register for 1ms interrupt (approximately)
  OCR2A = 249;  // (16MHz / 64) / 1000 = 250 Hz, so OCR2A = 249 for a 1ms interval

  // Enable interrupt on Compare Match A
  TIMSK2 |= (1 << OCIE2A);
}

// Timer2 Compare Match A interrupt service routine
ISR(TIMER2_COMPA_vect) {
  // Set the flag to indicate the timer interrupt was triggered
  timerFlag = true;
}
