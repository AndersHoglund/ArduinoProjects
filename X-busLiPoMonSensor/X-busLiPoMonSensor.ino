// Spektrum X-BUS Telemetry 6S LiPo cell voltage sensor

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

#define I2C_SDA_PIN A4
#define I2C_SCL_PIN A5

#define IDENTIFIER  TELE_DEVICE_LIPOMON

#define NO_DATA 0xff
#define INT_NO_DATA 0xffff
#define UINT_NO_DATA_BE 0x7fff
#define UINT_NO_DATA_LE 0xff7f

#define CELLS       6

// Globals
// Resistor ladder as per LiPoMonResistorLadder.jpg
 double cell_const[] =
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

int sensorPin[CELLS] = {A0, A1, A2, A3, A6, A7};
double prevVoltage = 0.0;

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
  prevVoltage = 0.0;
  
  for (int i = 0; i < CELLS; i++)
  {
    // Read raw voltage from analog pin.
    int sensorValue = analogRead(sensorPin[i]);

    // ADC scaling
    double cellVoltage = sensorValue * (double) 5/1023;
//    double cellVoltage = sensorValue * (double) 0.00472199;

    // Scale reading to full voltage.
    cellVoltage *= cell_const[i];

    // Isolate current cell voltage.
    double tmp = cellVoltage;
    cellVoltage -= prevVoltage;
    prevVoltage = tmp;

    unsigned short int centiVoltage = cellVoltage * 100;

    if (cellVoltage > 1.0)
    {
      TmBuffer.lipomon.cell[i] = (UINT16)centiVoltage;
    }
    else
    {
      TmBuffer.lipomon.cell[i] = UINT_NO_DATA_BE;
    }
    // debug
//    TmBuffer.lipomon.cell[i] = 300 + i;
  }
  TmBuffer.lipomon.temp = 0;
}
