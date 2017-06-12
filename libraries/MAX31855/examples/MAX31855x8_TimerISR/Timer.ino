// The following works for the ATMEGA328, and posibly others.  Refer to the microcontroller's datasheet
// Assuming a 16MHz clock, we set the prescaler to 1024.  This gives a timer speed of 16,000,000 / 1024 = 15,625. This means
// the timer counts from 0 to 15,625 in one second.  We'd like to sample the MAX31855 every 125ms (0.125s) or 8 times per
// second so we set the compare register OCR1A to 15,625 * 0.125 = 1953.

// Create the temperature object, defining the pins used for communication
MAX31855 temp = MAX31855(MISO, CS, SCK);

// Store the temperatures as they are read
volatile float temps[8];


void initializeThermocoupleTimer(void) {
  cli();                                // Disable global interrupts
  ADMUX  = 0;                           // Initialize the MUX and enable ADC and set frequency
  ADCSRA = (1<<ADEN) | (1<<ADPS2); 
  TCCR1A = 0;                           // Initialize Timer1 TCCR1A register to 0
  TCCR1B = 0;                           // Same for TCCR1B
  OCR1A = 1953;                         // Set compare match register to desired timer count
  TCCR1B |= (1 << WGM12);               // Turn on CTC mode
  TCCR1B |= (1 << CS10) + (1 << CS12);  // Set CS10-CS12 for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);              // Enable timer compare interrupt
  sei();                                // Enable global interrupts
  
  // Initialize the temperatures
  for (int i=0; i< 8; i++)
    temps[i] = NO_MAX31855;
}


// Timer ISR
ISR(TIMER1_COMPA_vect)
{
  volatile static int thermocouple = 0;
  
  // The timer has fired.  It has been 0.125 seconds since the previous reading was taken
  // Take a thermocouple reading
  temps[thermocouple] = temp.readThermocouple(CELSIUS);
  
  // Change the multiplexer to read the next thermocouple
  thermocouple = ++thermocouple % 8;
  // Select the thermocouple
  digitalWrite(T0, thermocouple & 1? HIGH: LOW);
  digitalWrite(T1, thermocouple & 2? HIGH: LOW);
  digitalWrite(T2, thermocouple & 4? HIGH: LOW);
  // This routine will execute in 0.125 seconds time.  By then the MAX31855 will
  // have an accurate reading for the selected thermocouple
}


// Routine used by the main app to get temperatures
// This routine disables and then re-enables interrupts so that data corruption doesn't occur
// when a reading is read while it is in the middle of being updated by the ISR.
float getTemperature(int thermocouple) {
  float temperature;
  
  // Disable interrupts while reading values
  noInterrupts();
  
  // Get the reading
  if (thermocouple >=0 && thermocouple <= 7)
    temperature = temps[thermocouple];
  else
    temperature = temp.readJunction(CELSIUS);
  
  // Reenable interrupts
  interrupts();
  
  // Return the temperature
  return temperature;
}


