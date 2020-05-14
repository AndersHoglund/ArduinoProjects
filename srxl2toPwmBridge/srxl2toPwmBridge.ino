/* 
 *  Spektrum SRXL2 to PWM Bridge
 */
//#define USE_SOFTWARE_SERIAL

#include <Servo.h>
#ifdef USE_SOFTWARE_SERIAL
#include <SoftwareSerial.h>
#endif

#include "spm_srxl.h"

// Select SRXL2 input and output pins (need to be combined into one halfduplex line with a diod)

#ifdef USE_SOFTWARE_SERIAL
#define SRXL2_INPUT_PIN 2     //  -------o--------- SRXL2_DATA
                              //         |
#define SRXL2_OUTPUT_PIN 3    //  ---<|---
#else
//Hardware UART might also need some pullup
                              // VCC +--|===|--
                              //              |
#define SRXL2_INPUT_PIN 1     //  ------------o--------- SRXL2_DATA
                              //              |
#define SRXL2_OUTPUT_PIN 0    //  ---<|--------
#endif

#define SRXL2_PORT_BAUDRATE_DEFAULT 115200
#define SRXL2_FRAME_TIMEOUT 22 

// Spektrum channel order
#define THRO 0
#define AILE 1
#define ELEV 2
#define YAW  3
#define GEAR 4
#define AUX1 5
#define AUX2 6
#define AUX3 7
#define AUX4 8
#define AUX5 9

// Only available at 22ms frame rate.
#define AUX6 10
#define AUX7 11

// Select the PWM output pins
#define PWM_THRO_PIN 9
#define PWM_AUX2_PIN 8
#define PWM_AUX3_PIN 7
#define PWM_AUX4_PIN 6
#define PWM_AUX5_PIN 5

typedef struct
{
  int channel;
  int pin;
  int value;
  unsigned long prevTime;
  Servo device;
} pwm_t;

pwm_t pwms[] =
{
  {THRO, PWM_THRO_PIN, 1000, 0},
  {AUX2, PWM_AUX2_PIN, 1500, 0},
  {AUX3, PWM_AUX3_PIN, 1500, 0},
  {AUX4, PWM_AUX4_PIN, 1500, 0},
  {AUX5, PWM_AUX5_PIN, 1500, 0}
};

#ifdef USE_SOFTWARE_SERIAL
SoftwareSerial srxl2port(SRXL2_INPUT_PIN, SRXL2_OUTPUT_PIN); // RX, TX
#else
#define srxl2port Serial
#endif

unsigned long currentTime;
unsigned long prevPwmTime = 0;
const long pwmInterval = 22;

void setup()
{ 
  srxl2port.begin(SRXL2_PORT_BAUDRATE_DEFAULT);
    
  srxlInitDevice(SRXL_DEVICE_ID, SRXL_DEVICE_PRIORITY, SRXL_DEVICE_INFO, 0x01000001);// // Init the local SRXL device with the unique ID 32 bit 0x01000001 hexadezimal.
  srxlInitBus(0, 1, SRXL_SUPPORTED_BAUD_RATES);// Init the SRXL bus: The bus index must always be < SRXL_NUM_OF_BUSES -- in this case, it can only be 0 since we have only 1 bus.

  for (int i=0; i < sizeof(pwms)/sizeof(pwm_t); i++ )
  {
    pwms[i].device.attach(pwms[i].pin);
  }
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
    unsigned char c = srxl2port.read(); // 
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

  for (int i=0; i < sizeof(pwms)/sizeof(pwm_t); i++ )
  {   
    currentTime = millis();
    if (currentTime - pwms[i].prevTime >= pwmInterval)
    {
      pwms[i].prevTime = currentTime;
      pwms[i].device.writeMicroseconds(pwms[i].value);
    }
  }    
}

///////////////////////// SRXL2 channel interface //////////////////////////////

void userProvidedReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafe)
{
  for (int i=0; i < sizeof(pwms)/sizeof(pwm_t); i++ )
  {   
    // Get channel value and convert to 988 - 1500 - 2011 pwm range
    pwms[i].value = 988 + srxlChData.values[pwms[i].channel] >> 6;    // 16-bit to 10-bit range (0 - 1023)
  }
}

void uartSetBaud(uint8_t uart, uint32_t baudRate) // Automatic adjust SRXL2 baudrate. 
{
  // Not supported yet
}

void uartTransmit(uint8_t uart, uint8_t* pBuffer, uint8_t length)
{
  for (uint8_t i=0; i < length; i++)
  {
    srxl2port.write(pBuffer[i]);
  }
  srxl2port.flush();
}
