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

#define NULL_DATA 0
#define NO_DATA 0xff
#define INT_NO_DATA 0xffff
#define UINT_NO_DATA_BE 0x7fff
#define UINT_NO_DATA_LE 0xff7f

#define I2C_SDA_PIN A4
#define I2C_SCL_PIN A5

#define IDENTIFIER TELE_DEVICE_USER_16SU


// Globals
UN_TELEMETRY TmBuffer = {IDENTIFIER, 0, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA};

typedef union
{
  unsigned char raw[2];
  unsigned int value;
} endianBuff_u;


unsigned int SwapEndian(unsigned int i)
{
  endianBuff_u b;
  unsigned char temp;

  b.value = i;
  //Swap bytes
  temp     = b.raw[0];
  b.raw[0] = b.raw[1];
  b.raw[1] = temp;

  return(b.value);
}

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
  // Dummy test angle. Replace with real angle measurement
  INT16 AoA = 15; // Degrees


  TmBuffer.user_16SU.sField1 = SwapEndian(AoA);

  delay(1000);
}
