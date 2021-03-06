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

// select the analog voltage input pin
#define SENSOR_PIN A0    

// Define voltage divider resistor values
#define RESISTOR_HIGH (330.0) // kOhms
#define RESISTOR_LOW (33.0)

#define ADC_SCALE 5.0/1023.0
#define SCALE ((double)((RESISTOR_LOW + RESISTOR_HIGH) / RESISTOR_LOW) * ADC_SCALE)

#define I2C_SDA_PIN A4
#define I2C_SCL_PIN A5

//Select which TM frame to use
#define USE_ESC_FRAME
//#define USE_RPM_FRAME

#define TELE_DEVICE_ESC (0x20) // ESC
#define TELE_DEVICE_RPM (0x7e) // RPM

#define NO_DATA 0xff
#define INT_NO_DATA 0xffff
#define UINT_NO_DATA_BE 0x7fff
#define UINT_NO_DATA_LE 0xff7f

typedef union
{
  unsigned char raw[2];
  unsigned int value;
} endianBuff_u;


#ifdef USE_ESC_FRAME
#define IDENTIFIER  TELE_DEVICE_ESC
#endif

#ifdef USE_RPM_FRAME
#define IDENTIFIER  TELE_DEVICE_RPM
#endif


// Globals
int once = 1;

UN_TELEMETRY TmBuffer = {IDENTIFIER, 0, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA};


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

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);           // Debug LED and scope trigger
  
  Wire.begin(IDENTIFIER);         // join i2c bus with a slave address
  Wire.onRequest(requestEvent);   // register event
}

void loop()
{
  if (once)
  {
    once = 0;
    digitalWrite(13, HIGH);
  }

  // read the value from the ADC:  
  unsigned int sensorValue = analogRead(SENSOR_PIN);
  
  // compute real voltage
  double voltage = (double)(sensorValue) * SCALE;
  unsigned short int centiVoltage= (unsigned short int) (voltage * 100.0);

#ifdef USE_ESC_FRAME
  TmBuffer.esc.voltsInput = SwapEndian(centiVoltage);
  TmBuffer.esc.tempBEC    = UINT_NO_DATA_LE; // Odd...
#endif

#ifdef USE_RPM_FRAME
  TmBuffer.rpm.volts = SwapEndian(centiVoltage);
#endif

}
