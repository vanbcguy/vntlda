/*
  Vanbcguy VNT LDA Controller - based on "Standalone VNT Controller" by DMN - http://dmn.kuulalaakeri.org/vnt-lda/
  My current code: https://github.com/vanbcguy/vntlda
  - Rewritten PID loop
  - Support added for EGT probe with LDA control based on EGTs
  - Other various small changes

*/

#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <MAX31855.h>    // Use library written for a faster read time - https://github.com/engineertype/MAX31855
#include <ResponsiveAnalogRead.h>
#include <PID_v1.h>

// Also used in NVRAM data store magic header
const unsigned char versionString[] PROGMEM  = "DMN-Vanbcguy Boost Ctrl v3.2.3";

#define PIN_BUTTON A5
#define PIN_HEARTBEAT 13

#define PIN_MAP A1
#define PIN_TPS A0
#define PIN_EMP A2

#define PIN_RPM_TRIGGER 2
#define PIN_VNT_N75 11
#define PIN_AUX_N75 3

#define PIN_LCD 8

#define LCD_BAUD_RATE 115200

// Pins for the EGT MAX31855
#define doPin 4
#define csPin 5
#define clPin 6

ResponsiveAnalogRead map_read(PIN_MAP, false);  // no sleep on the MAP read; need the resolution
ResponsiveAnalogRead tps_read(PIN_TPS, true);
ResponsiveAnalogRead rpm_read(0, true);

// Set up the LCD pin
SoftwareSerial lcd = SoftwareSerial(0, PIN_LCD);

// Set up the thermocouple pins (Adafruit MAX31855 thermocouple interface)
MAX31855 temp(doPin, csPin, clPin );

#define EGT_COOL 165
#define EGT_WARN 700
#define EGT_ALARM 775
#define EGT_MAX_READ 1101

#define IDLE_MAX_RPM 1150
#define MIN_BOOST_SPOOLED 10 // kPa
#define PID_CUT_IN 1520 // rpm
#define TPS_CUT_IN 18 // ~ 7%


/* Scaling factor for your sensors - 255 divided by this should equal the full scale deflection of your sensor */
#define MAP_SCALING_KPA 0.977
#define EMP_SCALING_KPA 1.953

/* Change this if you need to adjust the scaling of the PID outputs - ie if you need finer control at smaller fractional numbers increase this
  or if you need to have large multipliers then decrease this */
#define PIDControlRatio 300

/* The resolution we use to calculate RPM - we are only going to calculate RPM ever 'n' number of teeth that pass by; otherwise we are going to have
  a jittery value.  Divide this value by the 'Teeth per Rotation' setting to know how many revolutions before we caculate RPM. */
#define rpmResolution 30

// Set loop delay times
#define SERIAL_DELAY 107 // ms
#define EXEC_DELAY 50 //ms
#define DISPLAY_DELAY 250 // ms
#define MAP_DELAY 10 //ms


// Calculate Average values
#define AVG_MAX 15

#define MAP_AXIS_TPS 0xDE
#define MAP_AXIS_RPM 0xAD
#define MAP_AXIS_KPA 0xDD
#define MAP_AXIS_CELSIUS 0xAA
#define MAP_AXIS_VOLTAGE 0xAB
#define MAP_AXIS_DUTY_CYCLE 0xAC
#define MAP_AXIS_RAW 0x0
#define MAP_AXIS_EGT 0xAE

/*
  MAP format:

  'M','2','D'   // D - interpolated maps, d - nearest neighbor
  xsize,ysize,x-axis-type,y-axis-type,output-type,
  data[xsize,ysize],
  lastX,lastY,lastRet // automatically filled when used mapLookup

*/

unsigned char auxMap[] = {
  'M', '2', 'D',
  0x6, 0x8, MAP_AXIS_RPM, MAP_AXIS_EGT, MAP_AXIS_DUTY_CYCLE, // 01 - new version
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
  60, 60, 60, 60, 210, 210,
  210, 210, 210, 210, 210, 210,
  00, 00, 00,                // lastX,lastY,lastRet
};


unsigned char boostRequest[] = {
  'M', '2', 'D',
  0xC, 0xB, MAP_AXIS_RPM, MAP_AXIS_TPS, MAP_AXIS_KPA, // 01 - new version
  0, 0, 0, 15, 34, 44, 46, 46, 46, 43, 39, 37,
  0, 5, 15, 27, 38, 49, 51, 51, 51, 48, 43, 41,
  0, 5, 20, 32, 42, 54, 57, 57, 57, 53, 48, 45,
  0, 5, 20, 40, 55, 60, 63, 63, 63, 59, 53, 50,
  0, 5, 22, 43, 60, 67, 70, 70, 70, 65, 59, 55,
  0, 5, 24, 47, 65, 74, 78, 78, 78, 72, 66, 61,
  0, 6, 27, 58, 75, 92, 98, 98, 98, 90, 83, 76,
  0, 7, 30, 70, 92, 110, 123, 123, 123, 113, 104, 95,
  0, 9, 33, 75, 112, 138, 154, 154, 154, 141, 130, 119,
  0, 11, 41, 87, 128, 173, 193, 193, 193, 176, 162, 149,
  0, 12, 45, 85, 145, 192, 214, 214, 214, 195, 180, 166,
  00, 00, 00,                // lastX,lastY,lastRet
};

unsigned char boostDCMax[] = {
  'M', '2', 'D',
  0x8, 0xB, MAP_AXIS_RPM, MAP_AXIS_TPS, MAP_AXIS_DUTY_CYCLE,
  0, 204, 204, 180, 155, 140, 120, 70,
  0, 204, 204, 180, 155, 140, 120, 70,
  0, 204, 204, 175, 155, 140, 120, 70,
  0, 204, 190, 160, 135, 130, 120, 70,
  0, 204, 185, 160, 135, 130, 120, 70,
  0, 204, 180, 160, 135, 130, 120, 70,
  0, 204, 180, 155, 135, 130, 120, 70,
  0, 204, 175, 150, 135, 130, 120, 70,
  0, 204, 175, 145, 135, 125, 120, 70,
  0, 204, 180, 145, 135, 125, 120, 70,
  0, 204, 185, 145, 135, 125, 120, 70,
  00, 00, 00,                // lastX,lastY,lastRet
};

unsigned char boostDCMin[] = {
  'M', '2', 'D',
  0x9, 0xB, MAP_AXIS_RPM, MAP_AXIS_TPS, MAP_AXIS_DUTY_CYCLE,
  0, 50, 50, 50, 50, 50, 50, 50, 50,
  0, 121, 110, 100, 91, 83, 75, 68, 50,
  0, 121, 110, 100, 91, 83, 75, 68, 50,
  0, 121, 110, 100, 91, 83, 75, 68, 50,
  0, 110, 100, 91, 83, 75, 68, 62, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  0, 99, 90, 70, 70, 70, 70, 65, 50,
  00, 00, 00,                // lastX,lastY,lastRet
};

unsigned char n75precontrolMap[] = {
  'M', '2', 'D',
  0xC, 0xB, MAP_AXIS_RPM, MAP_AXIS_TPS, MAP_AXIS_DUTY_CYCLE,
  0, 90, 90, 110, 110, 100, 90, 75, 62, 50, 50, 50,
  0, 204, 204, 204, 182, 163, 145, 130, 122, 115, 106, 70,
  0, 204, 204, 198, 165, 148, 137, 126, 118, 112, 102, 70,
  0, 204, 204, 176, 160, 142, 130, 117, 107, 102, 95, 70,
  0, 204, 204, 145, 124, 115, 109, 101, 94, 87, 82, 70,
  0, 204, 204, 145, 115, 112, 104, 94, 90, 85, 78, 70,
  0, 204, 204, 132, 111, 109, 103, 94, 87, 80, 76, 70,
  0, 204, 204, 132, 110, 108, 102, 94, 86, 80, 76, 70,
  0, 204, 204, 138, 107, 103, 98, 91, 85, 80, 76, 70,
  0, 204, 204, 142, 128, 119, 110, 103, 97, 89, 82, 70,
  0, 204, 204, 160, 150, 140, 132, 121, 110, 98, 86, 70,
  00, 00, 00,              // lastX,lastY,lastRet
};

const unsigned char statusString1[] PROGMEM  = " Active view: ";

#define OPTIONS_VANESOPENIDLE 1
#define OPTIONS_VNTOUTPUTINVERTED 2

// contains configurable data. Can be stored in eeprom
struct settingsStruct {
  int tpsMin;
  int tpsMax;
  int mapMin;
  int mapMax;
  int egtMin;
  int egtMax;
  int empMin;
  int empMax;
  int rpmMax;
  int rpmTeethsPerRotation;
  unsigned char mode;
  char options;
  int boostKp;
  int boostKi;
  int boostKd;
};

settingsStruct settings;

//  contains calculated output data. calculated each run of mainloop
struct controlsStruct {
  // inputs
  volatile int tpsInput;
  unsigned char tpsCorrected;
  volatile int mapInput;
  double mapCorrected;
  volatile int egtInput;
  unsigned char egtCorrected;
  volatile int empInput;
  unsigned char empCorrected;
  char mode; // operating mode

  // outputs

  double vntTargetPressure;
  unsigned char vntPositionRemapped;
  unsigned char vntPositionDC;
  int vntMinDc;
  int vntMaxDc;
  int n75precontrol;

  // calculated value
  volatile int rpmActual;
  volatile unsigned char rpmCorrected;
  unsigned char statusBits;

  bool idling;
  int temp1;

  unsigned char auxOutput;

  float boostCalculatedP;
  float boostCalculatedI;
  float boostCalculatedD;

  double pidOutput;

  unsigned long lastTime;
  float lastInput;
};

controlsStruct controls;


double Kp;
double Ki;
double Kd;

// set up VNT PID control
PID vntPid(&controls.mapCorrected, &controls.pidOutput, &controls.vntTargetPressure, Kp, Ki, Kd, P_ON_E,DIRECT);

struct avgStruct {
  unsigned char pos;
  unsigned char size;
  volatile unsigned int avgData[AVG_MAX];
};

avgStruct mapAvg;

char buffer[100]; // general purpose buffer, mainly used for string storage when printing from flash
unsigned long lastPacketTime;
const unsigned char mapVisualitionHelp[] PROGMEM  = "Top Left is 0,0 (press: L - toggle live mode)";

unsigned char page = 0;
const char *pages[] = {
  "About", "Adaptation", "Actuator Fine-tune", "Edit map: boostRequest", "Edit map: boostDCMin", "Edit map: boostDCMax", "Edit map: n75preControl", "Edit map: Aux. device PWM map", "Output Tests"
};

unsigned char *editorMaps[] = {
  boostRequest, boostDCMin, boostDCMax, n75precontrolMap, auxMap
};

unsigned char clearScreen[] =  {
  27, '[', '2', 'J', 27, '[', 'H'
};

const unsigned char ANSIclearEol[] PROGMEM = {
  27, '[', 'K', 0
};

const unsigned char ANSIclearEolAndLf[] PROGMEM = {
  27, '[', 'K', '\r', '\n', 0
};
const unsigned char ANSIgoHome[] PROGMEM = {
  27, '[', '1', ';', '1', 'H', 0
};
const unsigned char ANSIclearEos[] PROGMEM = {
  27, '[', 'J', 0
};

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1:
        mode = 0x01;
        break;
      case 8:
        mode = 0x02;
        break;
      case 64:
        mode = 0x03;
        break;
      case 256:
        mode = 0x04;
        break;
      case 1024:
        mode = 0x05;
        break;
      default:
        return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    }
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1:
        mode = 0x01;
        break;
      case 8:
        mode = 0x02;
        break;
      case 32:
        mode = 0x03;
        break;
      case 64:
        mode = 0x04;
        break;
      case 128:
        mode = 0x05;
        break;
      case 256:
        mode = 0x06;
        break;
      case 1024:
        mode = 0x7;
        break;
      default:
        return;
    }
    // TCCR2A = 0xA3;
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


volatile unsigned int teethNo = 0;

void rpmTrigger() {
  // increase the tooth count whenever we see a tooth go by
  teethNo++;
}

unsigned long rpmMicros = 0;
unsigned long teethSeconds = 0;

void calcRpm() {
  if (teethNo > rpmResolution)
  {
    int rpm;

    detachInterrupt(0); // don't trigger increments while we're calculating

    teethSeconds = 60000000 / settings.rpmTeethsPerRotation;

    // teethSeconds is one second in microseconds / number of teeth per revolution - avoid overflowing by pre-dividing a second by the number of teeth
    rpm = (teethSeconds * teethNo) / (micros() - rpmMicros);

    // Set time to now, reset tooth count to zero to start incrementing again
    rpmMicros = micros();
    teethNo = 0;

    attachInterrupt(0, rpmTrigger, FALLING); // back to the daily grind

    // controls.rpmActual = (rpmSmoothing * rpm) + ((1.0-rpmSmoothing)*controls.rpmActual);

    rpm_read.update(rpm / 10);
    controls.rpmActual = rpm_read.getValue() * 10;

    if (controls.rpmActual > settings.rpmMax) {
      controls.rpmActual = 0;
    }
  }
}

void gotoXY(char x, char y) {
  Serial.print(F("\e["));
  Serial.print(y, DEC);
  Serial.print(F(";"));
  Serial.print(x, DEC);
  Serial.print(F("H"));
}

void setup_lcd() {
  // Put all the LCD setup stuff here, we will call this from
  // the "setup" loop

  lcd.begin(LCD_BAUD_RATE);

  // set the splash screen
  lcd.write(0xFE);
  lcd.write(0x40);
  strcpy_P(buffer, (PGM_P)&versionString);
  lcd.print(buffer);
  delay(10);


  // set the contrast, 200 is a good place to start, adjust as desired
  lcd.write(0xFE);
  lcd.write(0x50);
  lcd.write(200);
  delay(10);

  // set the brightness - we'll max it (255 is max brightness)
  lcd.write(0xFE);
  lcd.write(0x99);
  lcd.write(255);
  delay(10);

  // set background colour - r/g/b 0-255
  lcd.write(0xFE);
  lcd.write(0xD0);
  lcd.write(64);
  lcd.write(255);
  lcd.write((uint8_t)0);
  delay(10);

  // turn off cursors
  lcd.write(0xFE);
  lcd.write(0x4B);
  lcd.write(0xFE);
  lcd.write(0x54);

  // turn off autoscroll
  lcd.write(0xFE);
  lcd.write(0x52);

  // clear screen
  lcd.write(0xFE);
  lcd.write(0x58);
  delay(10);   // we suggest putting delays after each command

  // go 'home'
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(10);   // we suggest putting delays after each command

  vntPid.SetSampleTime(100);
}

void calcKp() {
  Kp = (float)(settings.boostKp) / PIDControlRatio;
}

void calcKi() {
  Ki = (float)(settings.boostKi) / PIDControlRatio;
}

void calcKd() {
  Kd = (float)(settings.boostKd) / PIDControlRatio;
}

void setup() {
  delay(1500);    // Wait for LCD to actually start up
  setup_lcd();

  Serial.begin(115200);
  Serial.print(F("Boot:"));

  pinMode(PIN_HEARTBEAT, OUTPUT); // DEBUG led

  pinMode(PIN_BUTTON, INPUT); // Reset switch
  digitalWrite(PIN_BUTTON, HIGH);  // activate pull up resistor

  pinMode(PIN_RPM_TRIGGER, INPUT); // Reset switch
  digitalWrite(PIN_RPM_TRIGGER, HIGH); // pullup for honeywell

  attachInterrupt(0, rpmTrigger, FALLING); // or rising!

  setPwmFrequency(PIN_VNT_N75, 128); // was 1024
  setPwmFrequency(PIN_AUX_N75, 128); // was 1024

  pinMode(PIN_VNT_N75, OUTPUT);
  pinMode(PIN_AUX_N75, OUTPUT);

  pinMode(PIN_TPS, INPUT);
  pinMode(PIN_MAP, INPUT);

  digitalWrite(PIN_TPS, LOW); // safety unconnected TPS
  digitalWrite(PIN_MAP, HIGH); // safety unconnected MAP

  // clear screen
  lcd.write(0xFE);
  lcd.write(0x58);

  lcd.print(F("Load:"));
  Serial.print(F("OK, Load:"));
  if (loadFromEEPROM(false) == false) {
    Serial.print(F("invalid conf."));
    lcd.print(F("INVALID CONF"));
    loadDefaults();
    delay(2000);
  }
  else {
    Serial.println(F("OK"));
    lcd.print(F("OK."));
    delay(500);
  }
  Serial.println(F("\r\n"));
  Serial.write(clearScreen, sizeof(clearScreen));

  mapAvg.size = AVG_MAX;

  //initial setup of kp/ki/kd
  calcKp();
  calcKi();
  calcKd();

  digitalWrite(PIN_HEARTBEAT, LOW);

  // set up screen
  layoutLCD();

  pageAbout(1); // force output
}

void loadDefaults() {
  memset(&settings, 0, sizeof(settingsStruct));
  settings.tpsMin = 85;
  settings.tpsMax = 970;
  settings.mapMin = 55;
  settings.mapMax = 975;
  settings.empMax = 1023;
  settings.egtMax = 970;
  settings.egtMin = 0;
  settings.rpmTeethsPerRotation = 4;
  settings.rpmMax = 6000;
  settings.options = 0;
  settings.boostKp = 220;
  settings.boostKi = 7;
  settings.boostKd = 15;
}


unsigned char mapValues(int raw, int mapMin, int mapMax) {
  if (raw < mapMin)
    return 0;
  if (raw >= mapMax)
    return 0xff;

  return map(raw, mapMin, mapMax, 0, 255);
}

unsigned char mapValuesSqueeze(int raw, int mapMin, int mapMax) {
  return map(raw, 0, 255, mapMin, mapMax);
}

unsigned char mapInterpolate(unsigned char p1, unsigned char p2, unsigned char pos) {
  return (p1 * (100 - pos) + p2 * pos) / 100;
}

unsigned char mapLookUp(unsigned char *mapData, unsigned char x, unsigned char y) {
  unsigned char isInterpolated = *(mapData + 2);
  unsigned char tableSizeX = *(mapData + 3);
  unsigned char tableSizeY = *(mapData + 4);
  unsigned char yPos;
  *(mapData + 8 + tableSizeX * tableSizeY) = x;
  *(mapData + 8 + tableSizeX * tableSizeY + 1) = y;

  if (tableSizeY) {
    yPos = y / (256 / (tableSizeY - 1));
  }
  else {
    yPos = 0;
  }
  unsigned char xPos = (x / (256 / (tableSizeX - 1)));
  int ofs = 8; // skip headers

  unsigned char p1 = *(mapData + ofs + (yPos * tableSizeX) + xPos);
  unsigned char p2 = *(mapData + ofs + (yPos * tableSizeX) + (((xPos + 1) >= tableSizeX) ? xPos : xPos + 1));
  unsigned char p3 = *(mapData + ofs + ((((yPos + 1) >= tableSizeY) ? yPos : yPos + 1) * tableSizeX) + xPos);
  unsigned char p4 = *(mapData + ofs + ((((yPos + 1) >= tableSizeY) ? yPos : yPos + 1) * tableSizeX) + (((xPos + 1) >= tableSizeX) ? xPos : xPos + 1));

  unsigned char ret;
  if (isInterpolated == 'D') {
    int amountX = (x % (256 / (tableSizeX - 1))) * (10000 / (256 / (tableSizeX - 1)));
    if (tableSizeY) {
      // 2D
      int amountY = (y % (256 / (tableSizeY - 1))) * (10000 / (256 / (tableSizeY - 1)));
      char y1 = mapInterpolate(p1, p2, amountX / 100);
      char y2 = mapInterpolate(p3, p4, amountX / 100);
      ret = mapInterpolate(y1, y2, amountY / 100);
    }
    else {
      // 1D
      ret = mapInterpolate(p1, p2, amountX / 100);
    }
  }
  else {
    ret = p1;
  }
  *(mapData + 8 + tableSizeX * tableSizeY + 2) = ret;
  return ret;
}


char mapDebugCharValue(unsigned char c) {
  if (c < 5) {
    return ' ';
  }
  else if (c < 20) {
    return '.';
  }
  else if (c < 60) {
    return ':';
  }
  else if (c < 128) {
    return '!';
  }
  else if (c < 180) {
    return 'o';
  }
  else if (c < 220) {
    return 'O';
  }
  else  {
    return '@';
  }
}


// Fetches and print string from flash to preserve some ram!
void printFromFlash(const unsigned char *str) {
  strcpy_P(buffer, (PGM_P)str);
  Serial.print(buffer);
}

int EEPROMwriteData(int offset, byte *ptr, int size) {
  int i;
  for (i = 0; i < size; i++)
    EEPROM.write(offset++, *(ptr++));
  return i;
}

int EEPROMreadData(int offset, byte *ptr, int size) {
  int i;
  for (i = 0; i < size; i++)
    *(ptr++) = EEPROM.read(offset++);
  return i;
}

void saveToEEPROM() {
  int ofs = 0;
  // write magic header
  strcpy_P(buffer, (PGM_P)&versionString);
  ofs += EEPROMwriteData(0, (byte*)&buffer, strlen(buffer));
  // write control struct
  ofs += EEPROMwriteData(ofs, (byte*)&settings, sizeof(settingsStruct));

  ofs += EEPROMwriteData(ofs, (byte*)&auxMap, sizeof(auxMap));
  ofs += EEPROMwriteData(ofs, (byte*)&boostRequest, sizeof(boostRequest));
  ofs += EEPROMwriteData(ofs, (byte*)&boostDCMin, sizeof(boostDCMin));
  ofs += EEPROMwriteData(ofs, (byte*)&boostDCMax, sizeof(boostDCMax));
  ofs += EEPROMwriteData(ofs, (byte*)&n75precontrolMap, sizeof(n75precontrolMap));

  printFromFlash(ANSIclearEolAndLf);
  Serial.print(ofs, DEC);
  Serial.print(F("SAVED "));
  Serial.print(ofs);
  Serial.print(F(" BYTES."));

  delay(1000);
}

bool loadFromEEPROM(bool force) {
  int ofs = 0;
  // if reset pin is active, no not load anything from eeprom
  if (digitalRead(PIN_BUTTON) == 0) {
    Serial.print(F("PIN_BUTTON active.."));
    delay(2000);
    return false;
  }
  // Check magic header to prevent data corruption of blank board or wrong version save file
  if (!force) {
    strcpy_P(buffer, (PGM_P)&versionString);
    for (ofs = 0; ofs < strlen(buffer); ofs++) {
      if (EEPROM.read(ofs) != buffer[ofs])
        return false;
    }
  }
  ofs = strlen(buffer);
  ofs += EEPROMreadData(ofs, (byte*)&settings, sizeof(settingsStruct));

  ofs += EEPROMreadData(ofs, (byte*)&auxMap, sizeof(auxMap));
  ofs += EEPROMreadData(ofs, (byte*)&boostRequest, sizeof(boostRequest));
  ofs += EEPROMreadData(ofs, (byte*)&boostDCMin, sizeof(boostDCMin));
  ofs += EEPROMreadData(ofs, (byte*)&boostDCMax, sizeof(boostDCMax));
  ofs += EEPROMreadData(ofs, (byte*)&n75precontrolMap, sizeof(n75precontrolMap));

  return true;
}


int toKpaMAP(int raw) {
  return raw * MAP_SCALING_KPA;
}

int toKpaEMP(int raw) {
  return raw * EMP_SCALING_KPA;
}

int toRpm(int raw) {
  return round(((float)settings.rpmMax / 255) * (float)raw);
}

int toEgt(int raw) {
  return round(((float)settings.egtMax / 255) * (float)raw);
}

int toTps(int raw) {
  // percent
  return int(raw / 2.55);
}

void printIntWithPadding(int val, unsigned char width, char padChar) {
  // print enough leading zeroes!
  memset(buffer, padChar, 30);
  // append string presentation of number to end
  itoa(val, buffer + 30, 10);
  // print string with given width
  Serial.print(buffer + 30 + strlen(buffer + 30) - width);
}

void printStringWithPadding(const unsigned char *str, unsigned char width, char padChar) {
  // print enough leading zeroes!
  memset(buffer, padChar, 30);
  // append string presentation of number to end
  strcpy_P(buffer + 30, (PGM_P)str);

  // print string with given width
  Serial.print(buffer + 30 + strlen(buffer + 30) - width);
}

void printPads(unsigned char n, char padChar) {
  memset(buffer, padChar, n);
  buffer[n] = 0;
  Serial.print(buffer);
}

// User interface functions
void pageHeader() {
  //Serial.write(clearScreen,sizeof(clearScreen));
  printFromFlash(ANSIgoHome);
  printFromFlash((const unsigned char*)versionString);
  Serial.print(F(" Mode:"));
  Serial.print(controls.mode, DEC);
  Serial.print(F(" "));
  printFromFlash(statusString1);
  Serial.print(pages[page]);
  printFromFlash(ANSIclearEolAndLf);
  printFromFlash(ANSIclearEolAndLf);
}

// Stored in the 32kB FLASH
const unsigned char aboutString1[] PROGMEM  = "Original code (c) 2011-2014 Juho Pesonen. Visit http://dmn.kuulalaakeri.org/dmn-boost-control/";
const unsigned char aboutString2[] PROGMEM  = "PID and EGT control (c) 2014-2017 Bryn Hughes. Visit https://github.com/vanbcguy/vntlda";
const unsigned char aboutString3[] PROGMEM  = "Press: <space> to jump next view, or press ...";
const unsigned char aboutString4[] PROGMEM  = "Questions? Or feedback? Send mail to linux@nashira.ca";

void pageAbout(char key) {

  if (key) {
    // update only if key pressed
    pageHeader();
    printFromFlash(aboutString1);
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(aboutString2);
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEolAndLf);

    printFromFlash(aboutString3);
    printFromFlash(ANSIclearEolAndLf);
    for (char i = 0; i < 8; i++) {
      printPads(11, ' ');
      Serial.print(F("<"));
      Serial.print(i, DEC);
      Serial.print(F("> "));
      Serial.print(pages[i]);
      printFromFlash(ANSIclearEolAndLf);
    }
    printFromFlash(ANSIclearEolAndLf);

    printFromFlash(aboutString4);
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEolAndLf);


    printFromFlash(ANSIclearEos);
  }
}

// TPS input: 200 Corrected: 0 (low:200, high:788);
const unsigned char statusRPM[] PROGMEM  = "RPM actual:";
const unsigned char statusCorrected[] PROGMEM  = " Corrected:";
const unsigned char statusTPSinput[] PROGMEM  = "TPS input:";
const unsigned char statusLow[] PROGMEM  = ":";
const unsigned char statusMAPinput[] PROGMEM  = "MAP input:";
const unsigned char statusVNTactOutput[] PROGMEM  = "VNT actuator output:";
const unsigned char statusHeader[] PROGMEM  = "Sensor values and adaptation map limits (l=live, y=save, p/P=load, R=reset)";
//                                          0123456789012345678901234567890123456789012345678901234567890123456789
const unsigned char statusTableHeader[] PROGMEM  = "        Raw val.  Corr.val. Map min.  Map max.  Mode";
const unsigned char statusRowTPS[] PROGMEM = "TPS";
const unsigned char statusRowMAP[] PROGMEM = "MAP";
const unsigned char statusRowEMP[] PROGMEM = "EMP";
const unsigned char statusRowEGT[] PROGMEM = "EGT";
const unsigned char statusRowRPM[] PROGMEM = "RPM";

//                                          0123456789012345678901234567890123456789012345678901234567890123456789
const unsigned char statusVNTtableStyleDC[] PROGMEM =  "Duty cycle";
const unsigned char statusVNTtableStyleMAP[] PROGMEM = "Target press.";
const unsigned char statusOpenAtIdle[] PROGMEM = "VNT Open blades when idling C";

const unsigned char statusNone[] PROGMEM = "-";

const unsigned char statusSelected[] PROGMEM = "[X] ";
const unsigned char statusUnSelected[] PROGMEM = "[ ] ";
const unsigned char statusVNTOutputInverted[] PROGMEM = "VNT Output Inverted J";
const unsigned char statusControlMethodDC[] PROGMEM =       "[X] BoostDCMin = output, [ ] PID, [ ] Simulate Actuator   M";
const unsigned char statusControlMethodPID[] PROGMEM =      "[ ] BoostDCMin = output, [X] PID, [ ] Simulate Actuator   M";
const unsigned char statusControlMethodActuator[] PROGMEM = "[ ] BoostDCMin = output, [ ] PID, [X] Simulate Actuator   M";

const unsigned char statusTemp1[] PROGMEM = "Temp1";
const unsigned char statusC[] PROGMEM = "°C";
const unsigned char statusOn[] PROGMEM = " (on)  ";
const unsigned char statusOff[] PROGMEM = " (off) ";
const unsigned char statusFooter[] PROGMEM = "To change adaptation value, press the letter after value.";
const unsigned char statusFooter2[] PROGMEM = "For example: q = decrease 'Map Low' for TPS / Q increase 'Map Low' for TPS";

char oldKey;
bool isLive = true;

void pageStatusAndAdaption(char key) {
  int x = 1;

  // Boost for hurry people!
  if (oldKey == key)
    x = 10;

  switch (key) {
    case 'l':
    case 'L':
      isLive = !isLive;
      break;
    case 'q':
      if (settings.tpsMin - x > 0) settings.tpsMin -= x;
      break;
    case 'Q':
      if (settings.tpsMin + x < settings.tpsMax) settings.tpsMin += x;
      break;
    case 'w':
      if (settings.tpsMax - x > settings.tpsMin) settings.tpsMax -= x;
      break;
    case 'W':
      if (settings.tpsMax + x < 1024) settings.tpsMax += x;
      break;
    case 'e':
      if (settings.egtMax + x < EGT_MAX_READ) settings.egtMax += x;
      break;
    case 'E':
      if (settings.egtMax - x > 0) settings.egtMax -= x;
      break;
    case 'a':
      if (settings.mapMin - x > 0) settings.mapMin -= x;
      break;
    case 'A':
      if (settings.mapMin + x < settings.mapMax) settings.mapMin += x;
      break;
    case 's':
      if (settings.mapMax - x > settings.mapMin) settings.mapMax -= x;
      break;
    case 'S':
      if (settings.mapMax + x < 1024) settings.mapMax += x;
      break;
    case 'f':
      if (settings.rpmTeethsPerRotation > 1) settings.rpmTeethsPerRotation -= 1;
      teethSeconds = 60000000 / settings.rpmTeethsPerRotation;
      break;
    case 'F':
      if (settings.rpmTeethsPerRotation < 99) settings.rpmTeethsPerRotation += 1;
      teethSeconds = 60000000 / settings.rpmTeethsPerRotation;
      break;
    case 'd':
      if (settings.rpmMax - 100 > 1000) settings.rpmMax -= 100;
      break;
    case 'D':
      if (settings.rpmMax + 100 < 9999) settings.rpmMax += 100;
      break;

    case 'c':
    case 'C':
      settings.options = settings.options ^ OPTIONS_VANESOPENIDLE;
      break;
    case 'm':
    case 'M':
    case 'j':
    case 'J':
      settings.options = settings.options ^ OPTIONS_VNTOUTPUTINVERTED;
      break;
    case 'y':
      saveToEEPROM();
      break;
    case 'p':
      loadFromEEPROM(false);
      break;
    case 'R':
      loadDefaults();
      break;
  }
  if (key) {
    lastPacketTime = millis();
  }
  oldKey = key;

  if (!key && !isLive)
    return;

  // update always if live
  pageHeader();

  printFromFlash(statusHeader);
  printFromFlash(ANSIclearEolAndLf);
  printFromFlash(ANSIclearEolAndLf);

  printFromFlash(statusTableHeader);
  printFromFlash(ANSIclearEolAndLf);

  printStringWithPadding(statusRowTPS, 7, ' ');
  printPads(1, ' ');
  printIntWithPadding(controls.tpsInput, 4, '0'); // RAW
  printPads(6, ' ');
  printIntWithPadding(controls.tpsCorrected, 3, '0'); // Corr.
  printPads(7, ' ');
  printIntWithPadding(settings.tpsMin, 4, '0'); // Map low
  printPads(1, ' ');
  Serial.print(F("Q"));
  printPads(4, ' ');
  printIntWithPadding(settings.tpsMax, 4, '0'); // Map high
  Serial.print(F(" W"));
  printFromFlash(ANSIclearEolAndLf);

  printStringWithPadding(statusRowMAP, 7, ' ');
  printPads(1, ' ');
  printIntWithPadding(controls.mapInput, 4, '0'); // RAW
  printPads(6, ' ');
  printIntWithPadding(controls.mapCorrected, 3, '0'); // Corr.
  printPads(7, ' ');
  printIntWithPadding(settings.mapMin, 4, '0'); // Map low
  printPads(1, ' ');
  Serial.print(F("A"));
  printPads(4, ' ');
  printIntWithPadding(settings.mapMax, 4, '0'); // Map high
  Serial.print(F(" S"));
  printFromFlash(ANSIclearEolAndLf);

  printStringWithPadding(statusRowRPM, 7, ' ');
  printPads(1, ' ');
  printIntWithPadding(controls.rpmActual, 4, '0'); // RAW
  printPads(6, ' ');
  printIntWithPadding(controls.rpmCorrected, 3, '0'); // Corrected
  printPads(17, ' ');
  printIntWithPadding(settings.rpmMax, 4, '0');
  Serial.print(F(" D"));
  printPads(4, ' ');
  Serial.print(F("No.teeths="));
  printIntWithPadding(settings.rpmTeethsPerRotation, 2, '0');
  Serial.print(F(" F"));

  printFromFlash(ANSIclearEolAndLf);

  printStringWithPadding(statusTemp1, 7, ' ');
  printPads(1, ' ');
  printIntWithPadding(controls.temp1, 4, ' ');
  printFromFlash(statusC);
  printPads(3, ' ');
  Serial.print(F("Max EGT="));
  printIntWithPadding(settings.egtMax, 3, ' ');
  Serial.print(F(" E"));

  printFromFlash(ANSIclearEolAndLf);

  printFromFlash(ANSIclearEolAndLf);
  printFromFlash(ANSIclearEolAndLf);

  printPads(8, ' ');

  if (settings.options & OPTIONS_VANESOPENIDLE) {
    printFromFlash(statusSelected);
  }
  else {
    printFromFlash(statusUnSelected);
  }
  printFromFlash(statusOpenAtIdle);

  printFromFlash(ANSIclearEolAndLf);

  printPads(8, ' ');
  if (settings.options & OPTIONS_VNTOUTPUTINVERTED) {
    printFromFlash(statusSelected);
  }
  else {
    printFromFlash(statusUnSelected);
  }
  printFromFlash(statusVNTOutputInverted);
  printFromFlash(ANSIclearEolAndLf);

  printFromFlash(ANSIclearEolAndLf);
  printFromFlash(ANSIclearEolAndLf);

  printFromFlash(statusFooter);
  printFromFlash(ANSIclearEolAndLf);

  printFromFlash(statusFooter2);

  printFromFlash(ANSIclearEos);
}


const unsigned char statusOutput1[] PROGMEM = "Output tests:";
const unsigned char statusOutput2[] PROGMEM = "<Q> Set output VNT output to Map min. for 2 seconds, value=";
const unsigned char statusOutput3[] PROGMEM = "<W> Set output VNT output to Map max. for 2 seconds, value=";
const unsigned char statusOutput4[] PROGMEM = "<E> Sweep output VNT output between min & max";

void pageOutputTests(char key) {

  if (key) {
    switch (key) {
      case 'q':
      case 'Q':
        controls.vntPositionRemapped = 0;
        updateOutputValues();
        updateLCD();
        delay(2000);
        break;
      case 'w':
      case 'W':
        controls.vntPositionRemapped = 255;
        updateOutputValues();
        updateLCD();
        delay(2000);
        break;
      case 'e':
      case 'E':
        for (controls.vntPositionRemapped = 0;
             controls.vntPositionRemapped < 255;
             controls.vntPositionRemapped++) {
          updateOutputValues();
          updateLCD();
          delay(20);
        }
        for (controls.vntPositionRemapped = 255;
             controls.vntPositionRemapped > 0;
             controls.vntPositionRemapped--) {
          updateOutputValues();
          updateLCD();
          delay(20);
        }
        break;
    }
    pageHeader();

    printFromFlash(statusOutput1);
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(statusOutput2);
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(statusOutput4);
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEos);
  }
}

const unsigned char exportConf[] PROGMEM = "Configuration dump:";
const unsigned char exportVntMap[] PROGMEM = "VNT Map dump:";
const unsigned char exportLdaMap[] PROGMEM = "LDA Map dump:";
const unsigned char exportBoostDCMax[] PROGMEM = "VNT Max DC Map dump:";
const unsigned char exportBoostDCMin[] PROGMEM = "VNT Min DC Map dump:";

void pageExport(char key) {
  if (key) {
    pageHeader();
    printFromFlash(exportConf);
    printFromFlash(ANSIclearEolAndLf);
    Serial.print(F("!AA"));
    for (int i = 0; i < sizeof(settingsStruct); i++) {
      if (i % 32 == 31)
        printFromFlash(ANSIclearEolAndLf);
      unsigned char v = (unsigned char) * (i + ((unsigned char*)&settings));
      if (v < 16)
        Serial.print(F("0"));
      Serial.print(v, HEX);
    }
    Serial.print(F("!"));
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEolAndLf);

    printFromFlash(exportVntMap);
    printFromFlash(ANSIclearEolAndLf);
    Serial.print(F("!AA"));
    for (int i = 0; i < sizeof(boostRequest); i++) {
      if (i && i % 16 == 0)
        printFromFlash(ANSIclearEolAndLf);
      unsigned char v = (unsigned char) * (i + ((unsigned char*)&boostRequest));
      if (v < 16)
        Serial.print(F("0"));
      Serial.print(v, HEX);
    }
    Serial.print(F("!"));
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEolAndLf);

    printFromFlash(exportBoostDCMax);
    printFromFlash(ANSIclearEolAndLf);
    Serial.print(F("!AB"));
    for (int i = 0; i < sizeof(boostDCMax); i++) {
      if (i && i % 16 == 0)
        printFromFlash(ANSIclearEolAndLf);
      unsigned char v = (unsigned char) * (i + ((unsigned char*)&boostDCMax));
      if (v < 16)
        Serial.print(F("0"));
      Serial.print(v, HEX);
    }
    Serial.print(F("!"));
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEolAndLf);

    printFromFlash(exportBoostDCMin);
    printFromFlash(ANSIclearEolAndLf);
    Serial.print(F("!AC"));
    for (int i = 0; i < sizeof(boostDCMin); i++) {
      if (i && i % 16 == 0)
        printFromFlash(ANSIclearEolAndLf);
      unsigned char v = (unsigned char) * (i + ((unsigned char*)&boostDCMin));
      if (v < 16)
        Serial.print(F("0"));
      Serial.print(v, HEX);
    }
    Serial.print(F("!"));
    printFromFlash(ANSIclearEolAndLf);
    printFromFlash(ANSIclearEolAndLf);

    printFromFlash(exportLdaMap);
    printFromFlash(ANSIclearEolAndLf);
    Serial.print(F("!AD"));
    for (int i = 0; i < sizeof(auxMap); i++) {
      if (i && i % 16 == 0)
        printFromFlash(ANSIclearEolAndLf);
      unsigned char v = (unsigned char) * (i + ((unsigned char*)&auxMap));
      if (v < 16)
        Serial.print(F("0"));
      Serial.print(v, HEX);
    }
    Serial.print(F("!"));

    printFromFlash(ANSIclearEolAndLf);

    printFromFlash(ANSIclearEos);
  }
}

unsigned int execTimeRead = 0;
unsigned int execTimeAct = 0;
unsigned int execTimeLcd = 0;

void pageDataLogger(char key) {
  Serial.print(toKpaMAP(controls.mapCorrected), DEC);
  Serial.print(F(","));
  Serial.print(toKpaMAP(controls.vntTargetPressure), DEC);
  Serial.print(F(","));
  Serial.print(controls.vntPositionRemapped, DEC);
  Serial.print(F(","));
  Serial.print(controls.tpsCorrected, DEC);
  Serial.print(F(","));
  Serial.print(controls.rpmActual, DEC);
  Serial.print(F(","));
  Serial.print(controls.temp1, DEC);
  Serial.print(F(","));
  Serial.print(controls.boostCalculatedP, DEC);
  Serial.print(F(","));
  Serial.print(controls.boostCalculatedI, DEC);
  Serial.print(F(","));
  Serial.print(controls.boostCalculatedD, DEC);
  Serial.print(F(","));
  Serial.print(controls.pidOutput, DEC);
  Serial.print(F(","));
  Serial.print(controls.n75precontrol, DEC);
  Serial.print(F(","));
  Serial.print(controls.vntMinDc, DEC);
  Serial.print(F(","));
  Serial.print(controls.vntMaxDc, DEC);
  Serial.print(F(","));
  Serial.print(controls.rpmCorrected, DEC);
  Serial.print(F(","));
  Serial.print(controls.mode, DEC);
  Serial.print(F(","));
  Serial.print(controls.auxOutput, DEC);
  Serial.print(F(","));
  Serial.print(millis() / 10, DEC);
  Serial.println();
}

void printMapAxis(unsigned char axisType, unsigned char idx, bool verbose) {
  switch (axisType) {
    case MAP_AXIS_RPM:
      Serial.print(toRpm(idx), DEC);
      if (verbose) Serial.print(F(" RPM"));
      break;
    case MAP_AXIS_TPS:
      Serial.print(toTps(idx), DEC);
      if (verbose) Serial.print(F("% TPS"));
      break;
    case MAP_AXIS_KPA:
      Serial.print(toKpaMAP(idx), DEC);
      if (verbose) Serial.print(F(" kPa"));
      break;
    case MAP_AXIS_CELSIUS:
      Serial.print(idx - 64, DEC);
      if (verbose) Serial.print(F(" °C"));
      break;
    case MAP_AXIS_EGT:
      Serial.print(toEgt(idx), DEC);
      if (verbose) Serial.print(F(" °C"));
      break;
    default:
      Serial.print(idx, DEC);
      if (verbose) Serial.print(F(" Raw"));
  }
}
struct mapEditorDataStruct {
  char cursorX;
  char cursorY;
  unsigned char lastX;
  unsigned char lastY;
  char currentMap;
  unsigned char clipboard;
}
mapEditorData;


const unsigned char mapCurrentOutput[] PROGMEM = "Current output:";
const unsigned char mapEditorHelp[] PROGMEM = "Press: cursor keys to move, - / + dec/inc, c/v copy/paste cell, y save";

void pageMapEditor(unsigned char mapIdx, int key, boolean showCurrent = false) {
  unsigned char *mapData = editorMaps[mapIdx];
  unsigned char tableSizeX = *(mapData + 3);
  unsigned char tableSizeY = *(mapData + 4);
  unsigned char axisTypeX = *(mapData + 5);
  unsigned char axisTypeY = *(mapData + 6);
  unsigned char axisTypeResult = *(mapData + 7);
  unsigned char lastXpos = *(mapData + 8 + tableSizeX * tableSizeY);
  unsigned char lastYpos = *(mapData + 8 + tableSizeX * tableSizeY + 1);
  unsigned char lastValue = *(mapData + 8 + tableSizeX * tableSizeY + 2);

  const char xPad = 5;
  const char xSpace = 7;
  const char yPad = 5;
  const char ySpace = 2;

  switch (key) {
    case -2:
      gotoXY(xPad + xSpace + mapEditorData.cursorX * xSpace + 1, yPad + ySpace + mapEditorData.cursorY * ySpace);
      printPads(xSpace - 2, ' ');
      gotoXY(xPad + xSpace + mapEditorData.cursorX * xSpace + 1, yPad + ySpace + mapEditorData.cursorY * ySpace);
      printMapAxis(axisTypeResult, *(mapData + 8 + mapEditorData.cursorX + mapEditorData.cursorY * tableSizeX), 0);

      return;
      break;
    case -1:
      // erase cursor
      gotoXY(xPad + xSpace + mapEditorData.cursorX * xSpace, yPad + ySpace + mapEditorData.cursorY * ySpace);
      Serial.print(F(" "));
      gotoXY(xPad + xSpace + mapEditorData.cursorX * xSpace + xSpace - 1, yPad + ySpace + mapEditorData.cursorY * ySpace);
      Serial.print(F(" "));
      return;
      break;
    case 'c':
      mapEditorData.clipboard = *(mapData + 8 + mapEditorData.cursorX + mapEditorData.cursorY * tableSizeX);
      return;
      break;
    case 'v':
      (*(mapData + 8 + mapEditorData.cursorX + mapEditorData.cursorY * tableSizeX)) = mapEditorData.clipboard;
      pageMapEditor(mapIdx, -2);
      return;
      break;
    case 'h':
      pageMapEditor(mapIdx, -1);
      if (mapEditorData.cursorX > 0)
        mapEditorData.cursorX--;
      pageMapEditor(mapIdx, 0);
      return;
      break;
    case 'l':
      pageMapEditor(mapIdx, -1);
      if (mapEditorData.cursorX < tableSizeX - 1)
        mapEditorData.cursorX++;
      pageMapEditor(mapIdx, 0);
      return;
      break;
    case 'k':
      pageMapEditor(mapIdx, -1);
      if (mapEditorData.cursorY > 0)
        mapEditorData.cursorY--;
      pageMapEditor(mapIdx, 0);
      return;
      break;
    case 'j':
      pageMapEditor(mapIdx, -1);
      if (mapEditorData.cursorY < tableSizeY - 1)
        mapEditorData.cursorY++;
      pageMapEditor(mapIdx, 0);
      return;
      break;
    case '+':
      if (*(mapData + 8 + mapEditorData.cursorX + mapEditorData.cursorY * tableSizeX) < 0xff)
        (*(mapData + 8 + mapEditorData.cursorX + mapEditorData.cursorY * tableSizeX))++;
      pageMapEditor(mapIdx, -2);
      return;
      break;
    case '-':
      if (*(mapData + 8 + mapEditorData.cursorX + mapEditorData.cursorY * tableSizeX) > 0)
        (*(mapData + 8 + mapEditorData.cursorX + mapEditorData.cursorY * tableSizeX))--;
      pageMapEditor(mapIdx, -2);
      return;
      break;
    case 'y':
      saveToEEPROM();
    case 0:
      if (showCurrent) {
        // Current interpreted value
        gotoXY(xPad + xSpace, yPad + tableSizeY * ySpace + 2);
        printFromFlash(mapCurrentOutput);
        printMapAxis(axisTypeResult, lastValue, 1);
        printFromFlash(ANSIclearEolAndLf);
      }

      // update cursors only:
      gotoXY(2, yPad + ySpace + round((float)mapEditorData.lastY * (float)((float)(tableSizeY - 1) * (float)ySpace / 255)));
      Serial.print(F("  "));
      gotoXY(xPad + xSpace + round((float)mapEditorData.lastX * (float)((float)tableSizeX * (float)xSpace / 255)), 4);
      Serial.print(F(" "));

      mapEditorData.lastY = lastYpos;
      mapEditorData.lastX = lastXpos;

      gotoXY(2, yPad + ySpace + round((float)lastYpos * (float)((float)(tableSizeY - 1) * (float)ySpace / 255)));
      Serial.print(F(">>"));
      gotoXY(xPad + xSpace + round((float)lastXpos * (float)((float)tableSizeX * (float)xSpace / 255)), 4);
      Serial.print(F("v"));

      gotoXY(xPad + xSpace + mapEditorData.cursorX * xSpace, yPad + ySpace + mapEditorData.cursorY * ySpace);
      Serial.print(F(">"));
      gotoXY(xPad + xSpace + mapEditorData.cursorX * xSpace + xSpace - 1, yPad + ySpace + mapEditorData.cursorY * ySpace);
      Serial.print(F("<"));

      return;
      break;
    default:
      if (mapEditorData.currentMap != mapIdx) {
        mapEditorData.cursorX = 0;
        mapEditorData.cursorY = 0;
        mapEditorData.currentMap = mapIdx;
      }
      pageHeader();
      printFromFlash(ANSIclearEos);
      gotoXY(0, 3);
      printFromFlash(mapEditorHelp);
  }
  // Table X header

  for (int x = 0; x < tableSizeX; x++) {
    gotoXY(xPad + (1 + x)*xSpace, yPad);
    int idx = round((float)((255 / (float)(tableSizeX - 1))) * (float)x);
    printPads(1, ' ');
    printMapAxis(axisTypeX, idx, ((x == 0 || x == (tableSizeX - 1)) ? true : false));
  }
  gotoXY(xPad + xSpace, yPad + 1);
  printPads(tableSizeX * xSpace, '-');

  // Table Y header

  for (int y = 0; y < tableSizeY; y++) {
    gotoXY(xPad - 1, yPad + (1 + y)*ySpace);
    int idx = round((float)((255 / (float)(tableSizeY - 1))) * (float)y);

    printMapAxis(axisTypeY, idx, true);
    gotoXY(xPad + xSpace - 1, yPad + (1 + y)*ySpace);
    Serial.print(F("|"));
    if (y < tableSizeY - 1) {
      gotoXY(xPad + xSpace - 1, yPad + (1 + y)*ySpace + 1); // works for ySpace=2
      Serial.print(F("|"));
    }

  }
  for (int y = 0; y < tableSizeY; y++) {
    for (int x = 0; x < tableSizeX; x++) {
      gotoXY(xPad + (1 + x)*xSpace, yPad + (1 + y)*ySpace);
      printPads(1, ' ');
      //Serial.print(*(mapData+8+x*y),DEC);
      printMapAxis(axisTypeResult, *(mapData + 8 + x + y * tableSizeX), 0);
    }
  }

}

const unsigned char debugHeader[] PROGMEM = "Target pres.   Actual press.  Actuator pos.  RPM  TPS";
// 0123456789012345678901234567890123456789012345678901234567890123456789


void pageHelp(char key) {
  //pageHeader();
  //Serial.print("help!");
  //printFromFlash(ANSIclearEos);
  if (key) {
    printFromFlash(debugHeader);
    Serial.print(F("\r\n"));
    printIntWithPadding(toKpaMAP(controls.vntTargetPressure), 3, '0');
    printPads(12, ' ');
    printIntWithPadding(toKpaMAP(controls.mapCorrected), 3, '0');
    printPads(12, ' ');
    printIntWithPadding(controls.vntPositionDC, 3, '0');
    printPads(12, ' ');
    printIntWithPadding(controls.rpmActual, 4, '0');
    printPads(1, ' ');
    printIntWithPadding(controls.tpsCorrected, 4, '0');
    Serial.print(F("\r\n"));
  }
}

const unsigned char ServoFineTunePosition[] PROGMEM = "Act. pos.: ";
const unsigned char ServoFineTunePositionScale[] PROGMEM = "0% ----------- 25% ----------- 50% ----------- 75% -------- 100%";

const unsigned char ServoFineTuneChargePressureRequest[] PROGMEM = "Charge pressure, request:";
const unsigned char ServoFineTuneChargePressureActual[] PROGMEM = "Charge pressure, actual:";
const unsigned char ServoFineTuneTPS[] PROGMEM = "TPS:";
const unsigned char ServoOutputDC[] PROGMEM = "N75 Duty Cycle:";
const unsigned char ServoFineTuneP[] PROGMEM = "PID Kp:";
const unsigned char ServoFineTuneI[] PROGMEM = "PID Ki:";
const unsigned char ServoFineTuneD[] PROGMEM = "PID Kd:";

void visualizeActuator(char y) {
  gotoXY(1, y);

  printFromFlash(ServoFineTunePosition);
  // append string presentation of number to end
  strcpy_P(buffer, (PGM_P)ServoFineTunePositionScale);
  // Visualize actuator
  memset(buffer, '*', controls.vntPositionDC / 4);
  Serial.print(buffer);
  printFromFlash(ANSIclearEol);
  gotoXY(1, y + 1);
  printFromFlash(ANSIclearEol);
  gotoXY(1, y + 1);
  gotoXY(12 + controls.vntMaxDc / 4, y + 1);
  Serial.print(F("^Max"));
  gotoXY(12 + controls.vntMinDc / 4, y + 1);
  Serial.print(F("^Min"));
}

void pageServoFineTune(char key) {
  pageHeader();


  switch (key) {
    case 'p':
      if (settings.boostKp > 0) settings.boostKp--;
      calcKp();
      break;
    case 'P':
      if (settings.boostKp < 1000) settings.boostKp++;
      calcKp();
      break;
    case 'i':
      if (settings.boostKi > 0) settings.boostKi--;
      calcKi();
      break;
    case 'I':
      if (settings.boostKi < 500) settings.boostKi++;
      calcKi();
      break;
    case 'd':
      if (settings.boostKd > 0) settings.boostKd--;
      calcKd();
      break;
    case 'D':
      if (settings.boostKd < 255) settings.boostKd++;
      calcKd();
      break;
    case 'y':
      saveToEEPROM();
      break;
  }


  visualizeActuator(3);

  gotoXY(1, 5);

  printStringWithPadding(ServoFineTuneChargePressureRequest, 25, ' ');
  printPads(1, ' ');
  printIntWithPadding(toKpaMAP(controls.vntTargetPressure), 3, '0');
  Serial.print(F(" kPa"));
  printFromFlash(ANSIclearEolAndLf);

  printStringWithPadding(ServoFineTuneChargePressureActual, 25, ' ');
  printPads(1, ' ');
  printIntWithPadding(toKpaMAP(controls.mapCorrected), 3, '0');
  Serial.print(F(" kPa"));
  printFromFlash(ANSIclearEolAndLf);

  printStringWithPadding(ServoFineTuneTPS, 25, ' ');
  printPads(1, ' ');
  printIntWithPadding(controls.tpsCorrected, 3, '0');
  printFromFlash(ANSIclearEolAndLf);

  printStringWithPadding(ServoOutputDC, 25, ' ');
  printPads(1, ' ');
  printIntWithPadding(controls.vntPositionRemapped, 3, '0');
  printFromFlash(ANSIclearEolAndLf);


  printStringWithPadding(ServoFineTuneP, 25, ' ');
  printPads(1, ' ');
  printIntWithPadding(settings.boostKp, 3, '0');
  Serial.print(F(" P ("));
  Serial.print(controls.boostCalculatedP * (100 * settings.boostKp / PIDControlRatio));
  Serial.print(F(")"));
  printFromFlash(ANSIclearEolAndLf);

  printStringWithPadding(ServoFineTuneI, 25, ' ');
  printPads(1, ' ');
  printIntWithPadding(settings.boostKi, 3, '0');
  Serial.print(F(" I ("));
  Serial.print(controls.boostCalculatedI * (100 * settings.boostKi / PIDControlRatio));
  Serial.print(F(")"));
  printFromFlash(ANSIclearEolAndLf);


  printStringWithPadding(ServoFineTuneD, 25, ' ');
  printPads(1, ' ');
  printIntWithPadding(settings.boostKd, 3, '0');
  Serial.print(F(" D ("));
  Serial.print(controls.boostCalculatedD * (settings.boostKd / PIDControlRatio));
  Serial.print(F(")"));
  printFromFlash(ANSIclearEolAndLf);

  printFromFlash(ANSIclearEolAndLf);

  printFromFlash(ANSIclearEos);
}


int getFilteredAverage(struct avgStruct *a) {
  int minVal = 0;
  int maxVal = 255;
  long int avgAll = 0;

  for (int i = 0; i < a->size; i++) {

    if (a->avgData[i] < minVal) {
      minVal = a->avgData[i];
    }
    if (a->avgData[i] > maxVal) {
      maxVal = a->avgData[i];
    }

    avgAll += a->avgData[i];
  }
  avgAll = (int)(avgAll / a->size);
  return avgAll;

}

void readValuesTps() {
  tps_read.update();
  controls.tpsInput = tps_read.getValue();
}

void readValuesMap() {

  mapAvg.pos++;
  if (mapAvg.pos >= mapAvg.size)
    mapAvg.pos = 0;

  map_read.update();
  mapAvg.avgData[mapAvg.pos] = map_read.getValue();

  controls.mapInput = getFilteredAverage(&mapAvg);

}

void readValuesEgt() {
  int egtread;

  egtread = temp.readThermocouple(CELSIUS);

  if (egtread <= 0) {
    controls.temp1 = 0;
  }
  else if (egtread >= 10000) {        // MAX31855 library returns > 10000 when there was a problem reading
    controls.temp1 = controls.temp1;
  }
  else if (egtread >= settings.egtMax) {
    controls.temp1 = settings.egtMax;
  }
  else {
    controls.temp1 = egtread;
  }

}

void determineIdle() {
  if ( controls.tpsCorrected > 0 ) {
    controls.idling = false;
  }
  else if ( controls.rpmActual < IDLE_MAX_RPM ) {    // Accelerator is at zero and we are in the idle speed band
    controls.idling = true;
  }
  else {
    controls.idling = false;                         // Most likely coasting right now; continue proportional behavior
  }
}

void controlVNT() {

  double minControl;
  double maxControl;

  double toControlVNT;

  controls.rpmCorrected = mapValues(controls.rpmActual, 0, settings.rpmMax);
  controls.mapCorrected = mapValues(controls.mapInput, settings.mapMin, settings.mapMax);
  controls.tpsCorrected = mapValues(controls.tpsInput, settings.tpsMin, settings.tpsMax);

  controls.vntMaxDc = mapLookUp(boostDCMax, controls.rpmCorrected, controls.tpsCorrected);
  controls.vntMinDc = mapLookUp(boostDCMin, controls.rpmCorrected, controls.tpsCorrected);

  controls.n75precontrol = mapLookUp(n75precontrolMap, controls.rpmCorrected, controls.tpsCorrected);

  /* Look up the requested boost */
  controls.vntTargetPressure = mapLookUp(boostRequest, controls.rpmCorrected, controls.tpsCorrected);

  /* This is the available span of our DC - we can only go between min and max */
  minControl = controls.vntMinDc - controls.n75precontrol;  // this will be a negative number
  maxControl = controls.vntMaxDc - controls.n75precontrol;  // this will be a positive number

  if ( minControl > 0 ) {
    // Our MinDC map is higher than our precontrol map; oops
    minControl = 0;
  }

  if ( maxControl < 0 ) {
    // Our MaxDC map is lower than our precontrol map; oops
    maxControl = 0;
  }

  vntPid.SetOutputLimits(minControl, maxControl);
  vntPid.SetTunings(Kp, Ki, Kd);

  if ((controls.idling)) {
    // If we are at idle then we don't want any boost regardless of map

    controls.vntTargetPressure = 0;                    // Display zero target pressure on the LCD at idle
    controls.mode = 0;                                 // System status = idling
    controls.pidOutput = 0;

    vntPid.SetMode(MANUAL);                            // Disable PID controller at idle

    if (settings.options & OPTIONS_VANESOPENIDLE) {
      toControlVNT = minControl;
    } else {
      toControlVNT = maxControl;
    }

  }
  else if (controls.mapCorrected <= MIN_BOOST_SPOOLED || controls.rpmActual < PID_CUT_IN || controls.tpsCorrected < TPS_CUT_IN ) {
    // If the turbo hasn't spooled up yet we're going to end up winding up the control loop; the precontrol map
    // should be more than sufficient to get things spinning
    
    controls.mode = 1;                                // We haven't spooled, don't start PID yet
    controls.pidOutput = 0;

    vntPid.SetMode(MANUAL);
  }
  else {

    vntPid.SetMode(AUTOMATIC);

    vntPid.Compute();

    if ( controls.pidOutput == minControl ) {
      // We are at minimum
      controls.mode = 4;
    } else if ( controls.pidOutput == maxControl ) {
      // We are at maximum
      controls.mode = 3;
    } else {
      // Normal in-range running
      controls.mode = 2;
    }

  }

  toControlVNT = round(controls.pidOutput) + controls.n75precontrol;

  controls.vntPositionDC = toControlVNT;

  /* This loop should never ever be true - a 100% output should be diff between min and max + min which should equal max
    but I'm not quite ready to remove this */
  if (controls.vntPositionDC > controls.vntMaxDc)
    controls.vntPositionDC = controls.vntMaxDc;

  /* Display these as real numbers - will make the logs more useful as we can try different values */
  controls.boostCalculatedP = vntPid.GetKp();
  controls.boostCalculatedI = vntPid.GetKi();
  controls.boostCalculatedD = vntPid.GetKd();

  unsigned char finalPos;
  finalPos = controls.vntPositionDC;

  if (settings.options & OPTIONS_VNTOUTPUTINVERTED) {
    controls.vntPositionRemapped = 255 - finalPos;
  }
  else {
    controls.vntPositionRemapped = finalPos;
  }

  // Include the time we spent processing
  controls.lastTime = millis();
}

void controlEGT() {
  // EGT controls
  controls.egtCorrected = mapValues(controls.temp1, settings.egtMin, settings.egtMax);
  controls.auxOutput = mapLookUp(auxMap, controls.rpmCorrected, controls.egtCorrected);
}

void updateOutputValues() {
  // PWM output pins
  analogWrite(PIN_VNT_N75, controls.vntPositionRemapped);
  analogWrite(PIN_AUX_N75, controls.auxOutput);
}

void layoutLCD() {
  // Set up the LCD for later writing - clear the screen, put all the stuff that doesn't change up
  lcd.write(0xFE);
  lcd.write(0x58);
  delay(100);

  position_lcd(0, 0);
  lcd.print(F("000/000k 0000rpm"));

  position_lcd(0, 1);
  lcd.print(F("000C T000   A000"));
}


byte lcdFlipFlop = 0;

void updateLCD() {

  digitalWrite(PIN_HEARTBEAT, lcdFlipFlop); // Flash the onboard LED, confirms the program is running

  if (lcdFlipFlop == 1 ) {
    // Only update half the LCD each cycle. Allow more frequent updates without disturbing control loop.
    position_lcd(0, 0);
    printn_lcd(toKpaMAP(controls.mapCorrected), 3);
    lcd.print(F("/"));
    printn_lcd(toKpaMAP(controls.vntTargetPressure), 3);

    lcd.print(F("  "));
    printn_lcd(controls.rpmActual, 4);
    lcd.print(F("rpm"));
    lcdFlipFlop = 0;
  } else {
    position_lcd(0, 1);
    printn_lcd(controls.temp1, 3);
    lcd.print(F("C T"));
    printn_lcd(controls.tpsCorrected, 3);

    lcd.print(F("   A"));
    printn_lcd(controls.vntPositionRemapped, 3);
    lcdFlipFlop = 1;
  }

  if (controls.temp1 < EGT_COOL) {
    {
      // Make the screen green if it isn't already
      // set background colour - r/g/b 0-255
      lcd.write(0xFE);
      lcd.write(0xD0);
      lcd.write((uint8_t)0);
      lcd.write(50);
      lcd.write(255);
    }
  }
  else if (controls.temp1 < EGT_WARN) {
    {
      // Make the screen green if it isn't already
      // set background colour - r/g/b 0-255
      lcd.write(0xFE);
      lcd.write(0xD0);
      lcd.write(64);
      lcd.write(255);
      lcd.write((uint8_t)0);
    }
  }
  else if (controls.temp1 < EGT_ALARM) {
    {
      // Make the screen orange if it isn't already
      lcd.write(0xFE);
      lcd.write(0xD0);
      lcd.write(255);
      lcd.write(32);
      lcd.write((uint8_t)0);
    }
  }
  else {
    {
      // Make the screen red if it isn't already
      lcd.write(0xFE);
      lcd.write(0xD0);
      lcd.write(255);
      lcd.write((uint8_t)0);
      lcd.write((uint8_t)0);
    }
  }
}

void position_lcd(int col, int row) {
  // allows a simple way of specifying a particular column and row for the
  // cursor to move to on the LCD. Specify starting from 0,0
  col = col + 1;
  row = row + 1;
  lcd.write(0xFE);
  lcd.write(0x47);
  lcd.write(col);
  lcd.write(row);
}

void printn_lcd(int lcdnumber, int lcddigits) {
  int padding = 0;
  if (lcdnumber < 10) {
    padding = lcddigits - 1;
  }
  else if (lcdnumber < 100) {
    padding = lcddigits - 2;
  }
  else if (lcdnumber < 1000) {
    padding = lcddigits - 3;
  }
  int i;
  for (i = 0; i < padding; i++) {
    lcd.print(F("0"));
  }
  lcd.print(lcdnumber);
}

void displayPage(char page, char data) {
  switch (page) {
    case 1:
      pageStatusAndAdaption(data);
      break;
    case 2:
      pageServoFineTune(data);
      break;
    case 3:
      pageMapEditor(0, data);
      visualizeActuator(28);

      break;
    case 4:
      pageMapEditor(1, data);
      visualizeActuator(28);

      break;
    case 5:
      pageMapEditor(2, data);
      visualizeActuator(28);
      break;
    case 6:
      pageMapEditor(3, data);
      visualizeActuator(28);
      break;
    case 7:
      pageMapEditor(4, data);
      break;
    case 8:
      pageOutputTests(data);
      visualizeActuator(28);
      break;
    case 9:
      pageExport(data);
      break;
    case 10:
      pageDataLogger(data);
      break;
    case 0:
    default:
      pageAbout(data);
  }
}

bool freezeModeEnabled = false;

unsigned long serialLoop = 0;
unsigned long execLoop = 0;
unsigned long displayLoop = 0;
unsigned long mapLoop = 0;


void loop() {

  if ((millis() - mapLoop) >= MAP_DELAY) {
    readValuesMap();  // Read every loop; we're calculating an average to clean up noise.
    mapLoop = millis();
  }

  /* Actual execution will happen every EXEC_DELAY - this is where we do our actual calculations */
  if ((millis() - execLoop) >= EXEC_DELAY) {


    execTimeRead = millis();
    readValuesTps();
    readValuesEgt();
    execTimeRead = millis() - execTimeRead;


    execTimeAct = millis();
    // We will actually process our values and change actuators every EXEC_DELAY milliseconds
    if (freezeModeEnabled) {
      Serial.print(F("\rFREEZE "));
    }
    else {
      // update output values according to input
      calcRpm();
      determineIdle();
      controlVNT();
      controlEGT();
      updateOutputValues();
    }
    execLoop = millis();
    execTimeAct = execLoop - execTimeAct;
  }


  /* we are only going to actualy process every SERIAL_LOOP_DELAY milliseconds though we will read from our sensors every loop
    This way we can get high resolution readings from the sensors without waiting for the actual calculations to occur every
    single time */

  else if ((millis() - serialLoop) >= SERIAL_DELAY) {

    unsigned char data = 0;

    // User interface for configuration and monitoring
    if (Serial.available()) {
      data = Serial.read();
      if (data >= '0' && data <= '9') {
        page = data - '0';
      }
      else if (data == ':') {
        freezeModeEnabled = !freezeModeEnabled;
      }
      else if (data == '#') {
        page = 10;
      }
      else if (data == 27) {
        data = Serial.read();
        if (data == '[') {
          data = Serial.read();
          switch (data) {
            case 'A':
              data = 'k';
              break;
            case 'B':
              data = 'j';
              break;
            case 'C':
              data = 'l';
              break;
            case 'D':
              data = 'h';
              break;

            default:
              data = 0;
          }
        }
      }
    }
    displayPage(page, data);
    serialLoop = millis();
  }

  /* The LCD also takes a while to update; only do the update every DISPLAY_DELAY ms as we don't need a blur where numbers should be */

  else if ((millis() - displayLoop) >= DISPLAY_DELAY) {
    execTimeLcd = millis();
    // We will only update the LCD every DISPLAY_DELAY milliseconds
    updateLCD();
    displayLoop = millis();
    execTimeLcd = displayLoop - execTimeLcd;
  }
}














