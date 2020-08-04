// Spektrum SRXL2 to ESC PWM Brifge, mainly intended for as an ESC voltage telemetry sensor

  // All this works on an Arduino Nano with Software Serial and Arduino ProMini(3.3V/8Mhz) HW UART.
  // Bootloader could/should be removed, i.e. upload this sketch using an ISP programmer.
  // Could not make it work with FC6258HX and anArduino ProMini(5V/16Mhz), too fast baudrate and/or too high signal leels.

//#define USE_SOFTWARE_SERIAL
//#define HAVE_FPU
//¤define FLUSH_INPUT_BUFFER_AFTER_TRANSMIT
#define USE_GLOBAL_SRXL2_CHANNEL_DATA

#include <Servo.h>
#ifdef USE_SOFTWARE_SERIAL
#include <SoftwareSerial.h>
#endif

#include "spm_srxl.h"
#include "spektrumTelemetrySensors.h"

// Select SRXL2 input and output pins (need to be combined into one halfduplex line with a zener diod, BAT85 works)

#ifdef USE_SOFTWARE_SERIAL
#define SRXL2_INPUT_PIN 2     //  ------------o--------- SRXL2_DATA
                              //              |
#define SRXL2_OUTPUT_PIN 3    //  ---|<--------
#else
//Hardware UART might also need a pullup resistor, depending on SRXL2 hub/master. FC6250HX seems to lack PU. 3.3k or so works.
                              // VCC +--|===|--
                              //              |
#define SRXL2_INPUT_PIN 1     //  ------------o--------- SRXL2_DATA
                              //              |
#define SRXL2_OUTPUT_PIN 0    //  ---|<--------
#endif

#define SRXL2_PORT_BAUDRATE_DEFAULT 115200
#define SRXL2_FRAME_TIMEOUT 50


// Spektrum normal channel order
#define THROTTLE_MASK 1
#define THRO 0
#define AILE 1
#define ELEV 2
#define ROLL 3
#define GEAR 4
#define AUX1 5
#define AUX2 6
#define AUX3 7
#define AUX4 8
#define AUX5 9

// Only available at 22ms frame rate, not at 11ms.
#define AUX6 10
#define AUX7 11

// Select the PWM output pin
#define PWM_OUTPUT_PIN 9

// select the analog voltage input pin
#define SENSOR_PIN A0    

// Define voltage divider resistor values
#define RESISTOR_HIGH (330.0) // kOhms
#define RESISTOR_LOW (33.0)

#define ADC_SCALE 5.0/1023.0
#define SCALE ((double)((RESISTOR_LOW + RESISTOR_HIGH) / RESISTOR_LOW) * ADC_SCALE)

#ifdef HAVE_FPU
#define SRXL2_TO_PWM_SCALE  32768.0/600.0
#define SRXL2_TO_PWM_OFFSET 900
#else
#define SRXL2_TO_PWM_SHIFT    6  // SCALE 64 really, 16-bit to 10-bit range (0 - 1023)
#define SRXL2_TO_PWM_OFFSET 988
#endif

typedef union
{
  unsigned char raw[2];
  unsigned short int value;
} endianBuff_u;

#define IDENTIFIER  TELE_DEVICE_ESC
#ifdef USE_SOFTWARE_SERIAL
SoftwareSerial srxl2port(SRXL2_INPUT_PIN, SRXL2_OUTPUT_PIN); // RX, TX
#else
#define srxl2port Serial
#endif

Servo pwmDevice;      // create PWM (ESC/servo) object to control a servo
unsigned int pwmPos = 1000;    // variable to store the PWM servo position

int sensorValue = 0;  // variable to store the value coming from the sensor

unsigned long currentTime;
unsigned long prevPwmTime = 0;
const long pwmInterval = 20;

UN_TELEMETRY TmBuffer = {IDENTIFIER, 0, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA};

unsigned short int BigEndian(unsigned short int i)
{
  endianBuff_u b;

  b.raw[0] = (i >> 8) & 0xff;
  b.raw[1] = i & 0xff;
  return(b.value);
}

unsigned short int LittleEndian(unsigned short int i)
{
  endianBuff_u b;

  b.raw[1] = (i >> 8) & 0xff;
  b.raw[0] = i & 0xff;
  return(b.value);
}


unsigned short int getCentiVoltage()
{
  // read the value from the ADC:  
  sensorValue = analogRead(SENSOR_PIN);
  
  // compute real voltage
  double voltage = sensorValue * SCALE;
  unsigned short int centiVoltage= voltage *100;

  return(centiVoltage);
}

////////////////////////////////////////////////////////////////////////
///////////////////////// SRXL2 interface //////////////////////////////
////////////////////////////////////////////////////////////////////////

void userProvidedFillSrxlTelemetry(SrxlTelemetryData* pTelemetryData)
{
  memcpy(pTelemetryData, &TmBuffer, sizeof(TmBuffer));
}

void userProvidedReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafe)
{
  unsigned int throttleChannelData;

  if ( isFailsafe )
  {
    digitalWrite(13, HIGH);  // Illuminate failsafe and trigger scope
    digitalWrite(13, LOW);
  }

// Get throttle channel data
#ifdef USE_GLOBAL_SRXL2_CHANNEL_DATA
  throttleChannelData = srxlChData.values[THRO];
#else
  if(pChannelData->mask & THROTTLE_MASK)
  {
      throttleChannelData = pChannelData->values[THRO];
  }
#endif

// Convert throttle channel value to pwm servo value range
#ifdef HAVE_FPU
  pwmPos = SRXL2_TO_PWM_OFFSET + (throttleChannelData / SRXL2_TO_PWM_SCALE);
#else
  pwmPos = SRXL2_TO_PWM_OFFSET + (throttleChannelData >> SRXL2_TO_PWM_SHIFT);
#endif
}

void userProvidedHandleVtxData(SrxlVtxData* pVtxData)
{
  // Not supported
}

void uartSetBaud(uint8_t uart, uint32_t baudRate)
{
  // Not supported yet
}

void uartTransmit(uint8_t uart, uint8_t* pBuffer, uint8_t length)
{
  for (uint8_t i=0; i < length; i++)
  {
    srxl2port.write(pBuffer[i]);
  }

  // Flush Tx buffer
  srxl2port.flush();

#if FLUSH_INPUT_BUFFER_AFTER_TRANSMIT
  // Flush the serial input buffer (The packet we just sent might be comming back in, depending on halfduplex hardware solution).
  while (srxl2port.available() > 0) srxl2port.read();
#endif

}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(SRXL2_INPUT_PIN, INPUT_PULLUP);
  pinMode(SRXL2_OUTPUT_PIN, OUTPUT);
  
  pwmDevice.attach(PWM_OUTPUT_PIN);

  srxl2port.begin(SRXL2_PORT_BAUDRATE_DEFAULT);
//  srxl2port.begin(SRXL2_PORT_BAUDRATE_DEFAULT, SERIAL_8N2);

  srxlInitDevice(SRXL_DEVICE_ID, SRXL_DEVICE_PRIORITY, SRXL_DEVICE_INFO, 0x01000000);
  srxlInitBus(0, 1, SRXL_SUPPORTED_BAUD_RATES);

  //Debug LED off
  pinMode(13,OUTPUT);
  digitalWrite(13, LOW);
}

void loop()
{
  currentTime = millis();

  static unsigned long prevSerialRxTime = 0;

  // UART receive buffer
  static uint8_t rxBuffer[2 * SRXL_MAX_BUFFER_SIZE];
  static uint8_t rxBufferIndex = 0;

  if (currentTime - prevSerialRxTime > SRXL2_FRAME_TIMEOUT)
  {
    prevSerialRxTime = currentTime;
    rxBufferIndex = 0;
    srxlRun(0, SRXL2_FRAME_TIMEOUT);
  }

  if ( srxl2port.available() )
  {
    prevSerialRxTime = currentTime;
    unsigned char c = srxl2port.read();
    rxBuffer[rxBufferIndex++] = c;
  }

  if (rxBufferIndex >= 5)
  {
    if(rxBuffer[0] == SPEKTRUM_SRXL_ID)
    {
      uint8_t packetLength = rxBuffer[2];
      if (rxBufferIndex >= packetLength)
      {
        // Try to parse SRXL packet -- this internally calls srxlRun() after packet is parsed and reset timeout
        if (srxlParsePacket(0, rxBuffer, packetLength))
        {
          // Move any remaining bytes to beginning of buffer (usually 0)
          rxBufferIndex -= packetLength;
          memmove(rxBuffer, &rxBuffer[packetLength], rxBufferIndex);
        }
        else
        {
            rxBufferIndex = 0;
        }
      }
    }
  }

  unsigned short int v    = getCentiVoltage();
  TmBuffer.esc.voltsInput = BigEndian(v);
  TmBuffer.esc.tempBEC    = UINT_NO_DATA_LE; // Odd...

  if (currentTime - prevPwmTime >= pwmInterval)
  {
    prevPwmTime = currentTime;
    pwmDevice.writeMicroseconds(pwmPos);
  }
}
