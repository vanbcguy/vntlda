/*
 MAX31855 library example sketch
 
 The MAX31855 uses SPI to communicate, and requires 3 pins.  This example displays the junction
 (chip) temperature and the thermocouple temperature.
 
 Look for the MAX31855 breakout boards on www.whizoo.com.
 
 readJunction() and readThermocouple() will return the temperature, or one of these errors:
 #define FAULT_OPEN        10000  // No thermocouple
 #define FAULT_SHORT_GND   10001  // Thermocouple short to ground
 #define FAULT_SHORT_VCC   10002  // Thermocouple short to VCC
 #define NO_MAX31855       10003  // MAX31855 not communicating

 
 Note:  If you connect the thermocouple probe the wrong way around, the temperature will go up
 instead of down (and vice versa).  No problem, just reverse the terminals.
 
 Released under WTFPL license.
 
 13 May 2014 by Peter Easton
 
*/
#include <MAX31855.h>


// Pin connections to the Quad MAX31855 board  
#define MISO 2
#define SCK  3
#define CS0  4
#define CS1  5
#define CS2  6
#define CS3  7

// Create the temperature object, defining the pins used for communication
MAX31855 *temp[4] = {new MAX31855(MISO, CS0, SCK), new MAX31855(MISO, CS1, SCK), new MAX31855(MISO, CS2, SCK), new MAX31855(MISO, CS3, SCK)};

void setup() {
  // Display temperatures using the serial port
  Serial.begin(9600);
}


void loop (){
  for (int i=0; i<4; i++) {
    double temperature = (*temp[i]).readJunction(CELSIUS);
    if (temperature == NO_MAX31855)
      continue;
    Serial.print("J");
    Serial.print(i);
    Serial.print("=");
    printTemperature(temperature);
    
    temperature = (*temp[i]).readThermocouple(CELSIUS);
    if (temperature == NO_MAX31855)
      continue;
    Serial.print("T");
    Serial.print(i);
    Serial.print("=");
    printTemperature(temperature);
  }
  Serial.println();
  delay(1000);
}


// Print the temperature, or the type of fault
void printTemperature(double temperature) {
  switch ((int) temperature) {
    case FAULT_OPEN:
      Serial.print("FAULT_OPEN");
      break;
    case FAULT_SHORT_GND:
      Serial.print("FAULT_SHORT_GND");
      break;
    case FAULT_SHORT_VCC:
      Serial.print("FAULT_SHORT_VCC");
      break;
    case NO_MAX31855:
      Serial.print("NO_MAX31855");
      break;
    default:
      Serial.print(temperature);
      break;
  }
  Serial.print(" ");
}




