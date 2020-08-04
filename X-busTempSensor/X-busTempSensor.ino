// Spektrum X-BUS Telemetry Temperature sensor 
// Warning EXPERIMENTAL Temp sensor scaling not determoned yet. Warning  

  // All this work if an Arduino Nano if the bootloader is removed, i.e. this sketch upload usinf an ISP programmer.
  // With the bootloader, it also works if Rx is powered up 5s after the Arduino Nano, fails if powered up at the same time. Clock stretching does not help.
  // Takes up to 4-5s to get Arduino Nano I2C up and running. Spektrum 8010T starts a single poll sequence after some 350mS, and thus gets no respons.
  // Spektrum says: "We don't recommend clock stretching with the T-series receivers."
  // Looks like SCL is driven high active, not by a pullup, on the scope. I.e. not I2C compliant.... Or?

#include <Wire.h>

// A few basic types Spektrum depends on
#define INT8 char
#define INT16 short int
#define INT32 long int
#define INT64 long long int

#define UINT8 unsigned char
#define UINT16 unsigned short int
#define UINT32 unsigned long int
#define UINT64 unsigned long long int

#include "spektrumTelemetrySensors.h"

// Define Temp sensor scaling values 
// Spektrum temp sensor scaling needs to be determined in some way. TBD TBD TBD
#define RESISTOR_HIGH (330.0) // kOhms
#define RESISTOR_LOW (33.0)
#define ADC_SCALE 5.0/1023.0
#define SCALE ((double)((RESISTOR_LOW + RESISTOR_HIGH) / RESISTOR_LOW) * ADC_SCALE)

#define I2C_SDA_PIN A4
#define I2C_SCL_PIN A5

#define NO_DATA 0xff
#define INT_NO_DATA 0xffff
#define UINT_NO_DATA_BE 0x7fff
#define UINT_NO_DATA_LE 0xff7f

//#define IDENTIFIER  TELE_DEVICE_TEMPERATURE   // legacy, DO NOT USE !
//#define IDENTIFIER  TELE_DEVICE_RPM           //Reserved, internal, can't be used via X-buz
//#define IDENTIFIER  TELE_DEVICE_MULTICYLINDER // Seems no support in either AR6610T or iX20 ???

// No real simple temp only device type available, lets try the Text Generator
#define IDENTIFIER  TELE_DEVICE_TEXTGEN
#define TEXTLINES 9     // Title plus 8 text lines
#define TEXTLINE_LEN 13
#define MOTORS 4

// Globals
int once = 1;
int temp[MOTORS+1]; // 1 based motor numbers.
const int sensor[MOTORS+1] = {0, A7, A6, A3, A2};

char textBuffer[TEXTLINES][TEXTLINE_LEN] = {"MOTOR TEMP", " ", " ", " ", " ", " ", " ", " ", " "};

UN_TELEMETRY TmBuffer = {IDENTIFIER, 0, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA};

void requestEvent()
{
  Wire.write(TmBuffer.raw, sizeof(TmBuffer) );
  TmBuffer.textgen.lineNumber++;
  if (TmBuffer.textgen.lineNumber >= TEXTLINES) TmBuffer.textgen.lineNumber=0;
}

void setup()
{
  TmBuffer.textgen.lineNumber = 0;
  
  Wire.begin(IDENTIFIER);         // join i2c bus with a slave address
  Wire.onRequest(requestEvent);   // register event
}

void loop()
{
  for (int motor=1; motor <= MOTORS; motor++ )
  {
    // read the value from the ADC:  
    int sensorValue = analogRead(sensor[motor]);

    // compute real temperature
    temp[motor] = (sensorValue) * SCALE;

    //DEBUG Test value only, until we have found out the SCALEing value for the thermistor. 
    temp[motor] = 34+motor;
    sprintf(textBuffer[motor], "M%d: %d C", motor, temp[motor]); 
  }
  
  strcpy(TmBuffer.textgen.text, textBuffer[TmBuffer.textgen.lineNumber]) ;
}
