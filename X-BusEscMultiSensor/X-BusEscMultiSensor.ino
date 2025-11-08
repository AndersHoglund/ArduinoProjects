// Spektrum X-BUS Telemetry ESC voltage sensor

  // All this work if an Arduino Nano if the bootloader is removed, i.e. this sketch upload usinf an ISP programmer.
  // With the bootloader, it also works if Rx is powered up 5s after the Arduino Nano, fails if powered up at the same time. Clock stretching does not help.
  // Takes up to 4-5s to get Arduino Nano I2C up and running. Spektrum 8010T starts a single poll sequence after some 350mS, and thus gets no respons.
  // Spektrum says: "We don't recommend clock stretching with the T-series receivers."
  // Looks like SCL is driven high active, not by a pullup, on the scope. I.e. not I2C compliant.... Or?

#include <Wire.h>

#include "spmTypes.h"
#include "spektrumTelemetrySensors.h"

#include "voltageSensor.h"
#include "rpmSensor.h"
#include "tempSensor.h"

#define I2C_SDA_PIN A4
#define I2C_SCL_PIN A5

typedef union
{
  unsigned char raw[2];
  unsigned int value;
} endianBuff_u;

//  Globals
int once = 1;

UN_TELEMETRY TmBuffer = {TELE_DEVICE_ESC, 0, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA};


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
  
  Wire.begin(TELE_DEVICE_ESC);    // join i2c bus with a slave address
  Wire.onRequest(requestEvent);   // register event
}

void loop()
{
  if (once)
  {
    once = 0;
    digitalWrite(13, HIGH);
  }

  double voltage = getVoltage();
  UINT16 rpm     = getRpm();
  UINT16 temp    = getTemp();

  TmBuffer.esc.RPM        = SwapEndian(rpm);
  TmBuffer.esc.voltsInput = SwapEndian(voltage*100); //Centivolts
  TmBuffer.esc.tempFET    = SwapEndian(temp*10);

}
