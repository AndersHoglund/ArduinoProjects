// Spektrum X-BUS Telemetry Temperature sensor 
// Warning EXPERIMENTAL Temp sensor scaling not determoned yet. Warning  

  // All this work if an Arduino Nano if the bootloader is removed, i.e. this sketch upload usinf an ISP programmer.
  // With the bootloader, it also works if Rx is powered up 5s after the Arduino Nano, fails if powered up at the same time. Clock stretching does not help.
  // Takes up to 4-5s to get Arduino Nano I2C up and running. Spektrum 8010T starts a single poll sequence after some 350mS, and thus gets no respons.
  // Spektrum says: "We don't recommend clock stretching with the T-series receivers."
  // Looks like SCL is driven high active, not by a pullup, on the scope. I.e. not I2C compliant.... Or?

//#define USE_DEVICE_RPM         // Can not get this to work. It is internal and reserved, can't be used via X-buz.
#define USE_DEVICE_MULTICYLINDER // Seems no support yet in iX20. Works fine with DX18g1 and AR6610T
//#define USE_DEVICE_TEXTGEN     // Works fine on both DX18 and iX20.

#define MOTORS 2

#include "Thermistor.h"
#include <Wire.h>

#define I2C_SDA_PIN A4
#define I2C_SCL_PIN A5

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

#define NULL_DATA 0x00
#define NO_DATA 0xff
#define INT_NO_DATA 0xffff
#define UCHAR_NO_DATA 0x7f
#define UINT_NO_DATA_BE 0x7fff
#define UINT_NO_DATA_LE 0xff7f

#define MAX_MOTORS 4
#if (MOTORS > MAX_MOTORS)
error
#endif

typedef union
{
  unsigned char raw[2];
  unsigned int value;
} endianBuff_u;

//#define IDENTIFIER  TELE_DEVICE_TEMPERATURE   // legacy, DO NOT USE !

#ifdef USE_DEVICE_RPM
#define IDENTIFIER  TELE_DEVICE_RPM
#define TEMP_SENSOR_PIN A7
#endif

#ifdef USE_DEVICE_MULTICYLINDER
#define IDENTIFIER  TELE_DEVICE_MULTICYLINDER


// Globals
int temp[MOTORS]; // 0 based motor numbers.
const int sensor[] = {A7, A6, A3, A2};
#endif

#ifdef USE_DEVICE_TEXTGEN
#define IDENTIFIER  TELE_DEVICE_TEXTGEN
#define TEXTLINES 9     // Title plus 8 text lines
#define TEXTLINE_LEN 13

// Globals
int temp[MOTORS+1]; // 1 based motor numbers.
const int sensor[] = {0, A7, A6, A3, A2};
char textBuffer[TEXTLINES][TEXTLINE_LEN] = {"MOTOR TEMP", " ", " ", " ", " ", " ", " ", " ", " "};
#endif

#ifdef USE_DEVICE_RPM
// Big endians
UN_TELEMETRY TmBuffer = {IDENTIFIER, 0, UCHAR_NO_DATA, NO_DATA, UCHAR_NO_DATA, NO_DATA, UCHAR_NO_DATA, NO_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA, NULL_DATA};
#else
UN_TELEMETRY TmBuffer = {IDENTIFIER, 0, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA};
#endif

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
#ifdef USE_DEVICE_TEXTGEN
  TmBuffer.textgen.lineNumber++;
  if (TmBuffer.textgen.lineNumber >= TEXTLINES) TmBuffer.textgen.lineNumber=0;
  strcpy(TmBuffer.textgen.text, textBuffer[TmBuffer.textgen.lineNumber]);
#endif
  Wire.write(TmBuffer.raw, sizeof(TmBuffer) );
}

void setup()
{
#ifdef USE_DEVICE_TEXTGEN
  TmBuffer.textgen.lineNumber = 0;
#endif
  
  Wire.begin(IDENTIFIER);         // join i2c bus with a slave address
  Wire.onRequest(requestEvent);   // register event
}

void loop()
{
#ifdef USE_DEVICE_RPM
  // read the value from the ADC/Thermistor
  int sensorValue = getTemperature(TEMP_SENSOR_PIN);
  sensorValue = (sensorValue * 9/5) + 32; // C to F

  //DEBUG Test value only, until we have found out the SCALEing value for the thermistor.
  //sensorValue = 101; //F

  TmBuffer.rpm.temperature = SwapEndian(sensorValue);
#endif

#ifdef USE_DEVICE_MULTICYLINDER
  for (int motor=0; motor < MOTORS; motor++ )
  {
    // read the value from the ADC:
    int sensorValue = getTemperature(sensor[motor]);
    temp[motor] = (sensorValue);

    // Range check
    if (temp[motor] < 30)
    {
      temp[motor] = 30;
    }

    if (temp[motor] > 284)
    {
      temp[motor] = 284;
    }

    TmBuffer.multiCylinder.temperature[motor] = temp[motor] - 30;
  }
#endif

#ifdef USE_DEVICE_TEXTGEN
  for (int motor=1; motor <= MOTORS; motor++ )
  {
    // read the value from the ADC:  
    int sensorValue = getTemperature(sensor[motor]);

    if (sensorValue > -20 && sensorValue < 250)
    {
      sprintf(textBuffer[motor], "M%d: %d C", motor, sensorValue);
    }
    else
    {
      sprintf(textBuffer[motor], "M%d: --", motor);
    }
  }
#endif
}
