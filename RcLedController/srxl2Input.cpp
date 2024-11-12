/*********** SRXL2 input driver **************/

/* Hardware UART must be used with an added schottky diode (BAT85 works) to create a single wire halfduplex interface.
  *
  *       Rx0 ------------o--------- SRXL2_DATA
  *                       |
  *       Tx1 ---|<--------
.
  */

#include <Arduino.h>
#include <stdint.h>

#include "RcLedController_conf.h"
#include "srxl2Input.h"

#ifdef USE_SOFTWARE_SERIAL
#error Software serial not supported
#endif

unsigned long prevPwmTime = 0;
const long pwmInterval = 22;

static uint8_t rcCh;
static uint16_t * pwmPtr;
static unsigned long prevSrxl2PacketTime = 0; // Last OK parsed SRXL2 packet

#if defined(ARDUINO_BLUEPILL_F103C6)
//HardwareSerial srxl2port(PA9);
HardwareSerial Serial1(PA9); // UART1_TX
#endif

void setupSRXL2()
{
  prevSrxl2PacketTime = millis();

  srxl2port.begin(SRXL2_PORT_BAUDRATE_DEFAULT);
  srxlInitDevice(SRXL_DEVICE_ID, SRXL_DEVICE_PRIORITY, SRXL_DEVICE_INFO, 0x01000001);
  srxlInitBus(0, 1, SRXL_SUPPORTED_BAUD_RATES);
}

void getSRXL2Pwm(unsigned long currentTime, uint8_t rcChannel, uint16_t * pwmValuePtr)
{
  static unsigned long prevSerialRxTime = 0;    // Last recieved serial data

  // UART receive buffer
  static uint8_t rxBuffer[2 * SRXL_MAX_BUFFER_SIZE];
  static uint8_t rxBufferIndex = 0;

  rcCh = rcChannel;
  pwmPtr = pwmValuePtr;

  if ((currentTime - prevSrxl2PacketTime) > (SRXL2_FRAME_TIMEOUT*50))
  {
    *pwmValuePtr = 0; // Signal time out no SRXL2 data recieved after 50 frame times.
    return;
  }

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
          prevSrxl2PacketTime = currentTime;
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
}


///////////////////////// SRXL2 channel interface //////////////////////////////

void userProvidedReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafe)
{ 
  // Get channel values and return ity on the supplied pointer.
  *pwmPtr = 988 + (srxlChData.values[rcCh] >> 6);    // 16-bit to 10-bit range (0 - 1024)
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
}
