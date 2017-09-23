// This library is derived from RocketScream.com's I2C MAX31855 library.
// Please support them from buying products from their web site.
// https://github.com/rocketscream/MAX31855/blob/master/MAX31855.cpp
//
// This is a library for the MAX31855 thermocouple IC
//
// Look for the MAX31855 breakout boards on www.whizoo.com.
//
// Written by Peter Easton (www.whizoo.com)
// Released under WTFPL license
//
// Change History:
// 13 May 2014        Initial Version

#include	"MAX31855.h"


MAX31855::MAX31855(int miso, int cs, int clk)
{
    _miso = miso;
    _cs = cs;
    _clk = clk;

    // MAX31855 data output pin
    // Use internal pullup to be able to detect no connection to the MAX31855
    pinMode(_miso, INPUT_PULLUP);
    // MAX31855 chip select input pin
    pinMode(_cs, OUTPUT);
    // MAX31855 clock input pin
    pinMode(_clk, OUTPUT);

    // Default output pins state
    digitalWrite(_cs, HIGH);
    digitalWrite(_clk, LOW);
}

/*******************************************************************************
* Name: readThermocouple
* Description: Read the thermocouple temperature either in Degree Celsius or
*			   Fahrenheit. Internally, the conversion takes place in the
*			   background within 100 ms. Values are updated only when the CS
*			   line is high.
*
* Argument  	Description
* =========  	===========
* 1. unit   	Unit of temperature required: CELSIUS or FAHRENHEIT
*
* Return		Description
* =========		===========
* temperature	Temperature of the thermocouple either in Degree Celsius or
*				Fahrenheit. If fault is detected, FAULT_OPEN, FAULT_SHORT_GND or
*				FAULT_SHORT_VCC will be returned. These fault values are outside
*				of the temperature range the MAX31855 is capable of.
*******************************************************************************/	
double	MAX31855::readThermocouple(unit_t	unit)
{
    unsigned long data;
    double temperature;

    // Initialize temperature
    temperature = 0;

    // Shift in 32-bit of data from MAX31855
    data = readData();

    // Is this a MAX31855?  Bits 3 and 17 must be zero
    if (data & 0x20008)
        return NO_MAX31855;

    // If fault is detected
    if (data & 0x00010000)
    {
        // Check for fault type (3 LSB)
        switch (data & 0x00000007)
        {
            // Open circuit
            case 0x01:
                temperature = FAULT_OPEN;
                break;

            // Thermocouple short to GND
            case 0x02:
                temperature = FAULT_SHORT_GND;
                break;

            // Thermocouple short to VCC	
            case 0x04:
                temperature = FAULT_SHORT_VCC;
                break;
        }
    }
    // No fault detected
    else
    {
        // Retrieve thermocouple temperature data and strip redundant data
        data = data >> 18;
        // Bit-14 is the sign
        temperature = (data & 0x00001FFF);

        // Check for negative temperature		
        if (data & 0x00002000)
        {
            // 2's complement operation
            // Invert
            data = ~data; 
            // Ensure operation involves lower 13-bit only
            temperature = data & 0x00001FFF;
            // Add 1 to obtain the positive number
            temperature += 1;
            // Make temperature negative
            temperature *= -1; 
        }

        // Convert to Degree Celsius
        temperature *= 0.25;

        // If temperature unit in Fahrenheit is desired
        if (unit == FAHRENHEIT)
        {
            // Convert Degree Celsius to Fahrenheit
            temperature = (temperature * 9.0/5.0)+ 32;
        }
    }
    return (temperature);
}

/*******************************************************************************
* Name: readJunction
* Description: Read the thermocouple temperature either in Degree Celsius or
*			   Fahrenheit. Internally, the conversion takes place in the
*			   background within 100 ms. Values are updated only when the CS
*			   line is high.
*
* Argument  	Description
* =========  	===========
* 1. unit   	Unit of temperature required: CELSIUS or FAHRENHEIT
*
* Return		Description
* =========		===========
* temperature	Temperature of the cold junction either in Degree Celsius or
*				Fahrenheit.
*
*******************************************************************************/
double	MAX31855::readJunction(unit_t	unit)
{
    double	temperature;
    unsigned long data;

    // Shift in 32-bit of data from MAX31855
    data = readData();

    // Is this a MAX31855?  Bits 3 and 17 must be zero
    if (data & 0x20008)
        return NO_MAX31855;

    // Strip fault data bits & reserved bit
    data = data >> 4;
    // Bit-12 is the sign
    temperature = (data & 0x000007FF);

    // Check for negative temperature
    if (data & 0x00000800)
    {
        // 2's complement operation
        // Invert
        data = ~data; 
        // Ensure operation involves lower 11-bit only
        temperature = data & 0x000007FF;
        // Add 1 to obtain the positive number
        temperature += 1;	
        // Make temperature negative
        temperature *= -1; 
    }

    // Convert to Degree Celsius
    temperature *= 0.0625;

    // If temperature unit in Fahrenheit is desired
    if (unit == FAHRENHEIT)
    {
        // Convert Degree Celsius to Fahrenheit
        temperature = (temperature * 9.0/5.0)+ 32;
    }

    // Return the temperature
    return (temperature);
}

/*******************************************************************************
* Name: readData
* Description: Shift in 32-bit of data from MAX31855 chip. Minimum clock pulse
*			   width is 100 ns. No delay is required in this case.
*
* Argument  	Description
* =========  	===========
* 1. NIL
*
* Return		Description
* =========		===========
* data			32-bit of data acquired from the MAX31855 chip.
*				
*******************************************************************************/
unsigned long MAX31855::readData()
{
    int bitCount;
    unsigned long data;

    // Clear data 
    data = 0;

    // Select the MAX31855 chip
    digitalWrite(_cs, LOW);

    // Shift in 32-bit of data
    for (bitCount = 31; bitCount >= 0; bitCount--)
    {
        digitalWrite(_clk, HIGH);

        // If data bit is high
        if (digitalRead(_miso))
        {
            // Need to type cast data type to unsigned long, else compiler will
            // truncate to 16-bit
            data |= ((unsigned long)1 << bitCount);
        }	

        digitalWrite(_clk, LOW);
    }

    // Deselect MAX31855 chip
    digitalWrite(_cs, HIGH);

    return(data);
}