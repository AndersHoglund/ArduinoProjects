// Spektrum X-BUS Telemetry ESC voltage sensor

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

// Define the analog voltage input pins
#define SENSOR_PINS (A0, A1, A2, A3, A6, A7)    
#define I2C_SDA_PIN A4
#define I2C_SCL_PIN A5

#define IDENTIFIER  TELE_DEVICE_LIPOMON

#define NO_DATA 0xff
#define INT_NO_DATA 0xffff
#define UINT_NO_DATA_BE 0x7fff
#define UINT_NO_DATA_LE 0xff7f

#define K           0.00472199
//#define K           5/1023
#define CELLS       6
#define MAX_CELLS   12

// Resistor ladder as per LiPoMonResistorLadder.jpg
double cell_const[MAX_CELLS] = 
{
  1.0000,
  2.1915,
  2.6970,
  4.1111,
  4.7333,
  6.6000,
  6.6000,
  7.8293,
  8.4667,
  9.2353,
  11.0000,
  11.0000
};

// Globals
int sensorPin[CELLS] = {SENSOR_PINS};

UN_TELEMETRY TmBuffer = {IDENTIFIER, 0, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA};

void requestEvent()
{
  Wire.write(TmBuffer.raw, sizeof(TmBuffer) );
}

void setup()
{
  Wire.begin(IDENTIFIER);         // join i2c bus with a slave address
  Wire.onRequest(requestEvent);   // register event
}

void loop()
{ 
  double battVoltage = 0.0;
  double prevVoltage = 0.0;
  
  for (int i = 0; i < CELLS; i++)
  {
    // Read raw voltage from analog pin.
    double cellVoltage = analogRead(sensorPin[i]) * K;
    
    // Scale reading to full voltage.
    cellVoltage *= cell_const[i];
    double tmp = cellVoltage;
    
    // Isolate current cell voltage.
    cellVoltage -= prevVoltage;
    battVoltage += cellVoltage;
    prevVoltage = tmp;

    if (cellVoltage > 2.0)
    {
      TmBuffer.lipomon.cell[i] = (UINT16)cellVoltage*100; // Centivolts
    }
    else
    {
      TmBuffer.lipomon.cell[i] = UINT_NO_DATA_BE;
    }
  }
  TmBuffer.lipomon.temp = 0;
}
