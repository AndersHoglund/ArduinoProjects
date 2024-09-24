//
#include <stdint.h>
#include <arduino.h>
#include <SoftwareSerial.h>

#include "RcLedController_conf.h"

#ifdef USE_SERIAL_RX_INPUT

#include "serialRxinput.hpp"

#ifdef USE_SOFTWARE_SERIAL
// Select Arduino input pin
#define INPUT_PIN A0
#define OUTPUT_PIN A1

SoftwareSerial serialRx(INPUT_PIN, OUTPUT_PIN); // RX, TX
#else
#define serialRx Serial
#endif

void setupSerialRx()
{
#ifdef USE_SOFTWARE_SERIAL
  pinMode(INPUT_PIN, INPUT);
#endif
  serialRx.begin(SPEKTRUM_BAUDRATE);
}

void getSerialRxPwm(unsigned long currentTime, uint8_t rcChannel, uint16_t * pwmValuePtr)
{
  static unsigned long lastSerialRxTime = 0;
  static unsigned char spekFrame[SPEK_FRAME_SIZE];
  static unsigned char spekFramePosition = 0;
  static bool rcFrameComplete = false;

  unsigned long spekChannelData[SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT];

  if (currentTime - lastSerialRxTime > SPEKTRUM_NEEDED_FRAME_INTERVAL)
  {
    spekFramePosition = 0;
  }

  if (spekFramePosition < SPEK_FRAME_SIZE && serialRx.available())
  {
    lastSerialRxTime = currentTime;

    unsigned char c = serialRx.read();
    spekFrame[spekFramePosition++] = c;
    
    if (spekFramePosition < SPEK_FRAME_SIZE)
    {
      rcFrameComplete = false;
    }
    else
    {
      rcFrameComplete = true;
    }
  }

  if (rcFrameComplete)
  {
    rcFrameComplete = false;

    // Get the RC control channel inputs
    for (int b = 3; b < SPEK_FRAME_SIZE; b += 2)
    {
      const unsigned char spekChannel = 0x0F & (spekFrame[b - 1] >> SPEK_CHAN_SHIFT);

      if (spekChannel < SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT)
      {
        spekChannelData[spekChannel] = ((spekFrame[b - 1] & SPEK_CHAN_MASK) << 8) + spekFrame[b];
      }
    }

    // Convert to PWM 1000 .. 1500 .. 2000 value range
    *pwmValuePtr = (988 + (spekChannelData[rcChannel] >> 1) );   // 2048 to 1024 resolution
  }
}
#endif