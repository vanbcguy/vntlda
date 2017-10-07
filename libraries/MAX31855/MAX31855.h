// Written by Peter Easton (www.whizoo.com)
// Released under WTFPL license
//
// Look for the MAX31855 breakout boards on www.whizoo.com.
//
// Change History:
// 13 May 2014        Initial Version

#ifndef MAX31855_H
#define MAX31855_H

#include "Arduino.h"

#define	FAULT_OPEN		10000  // No thermocouple
#define	FAULT_SHORT_GND	10001  // Thermocouple short to ground
#define	FAULT_SHORT_VCC	10002  // Thermocouple short to VCC
#define NO_MAX31855     10003  // MAX31855 not communicating

enum	unit_t
{
	CELSIUS,
	FAHRENHEIT
};

class	MAX31855
{
public:
    MAX31855(int, int, int);
	
    double	readThermocouple(unit_t	unit);
    double	readJunction(unit_t	unit);
    
private:
    unsigned long readData();
    int _miso, _cs, _clk;
};
#endif  // MAX31855_H