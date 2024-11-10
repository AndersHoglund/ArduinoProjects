/*
   Arduino 12 channel RC LED Controller

   Nav lights, ACL strobes, beacons and landing and reverse lights. See LedOutput.cpp for details.
   Using SRXL2 or PWM input, automatically detected.
   If you need SerialRx from a satellite Rx, or fixed single PWM input, checkout older versions from Git.
*/

#include "RcLedController_conf.h"

#include "srxl2Input.h"
#include "PWMinput.h"
#include "LedOutput.h"

// Ugly globals....
unsigned long currentTime;
uint16_t pwmInput = 1000;  // All lights off by default at power up

// Input types
#define NONE  0
#define SRXL2 1
#define PWM   2

/*************************** Initialize ***********************************/
void setup()
{
  setupSRXL2();
  setupPWM();
  setupLeds();
}

/*************************** Process ***********************************/
void loop()
{
  static uint8_t inputType = SRXL2;
  currentTime = millis();

  switch (inputType)
  {
    case NONE: // Should never happen, but .....
    {
      pwmInput = 1000;
      inputType++; // Next input type
      break;
    }

    case SRXL2:
    {
      getSRXL2Pwm(currentTime, LED_CONTROL_CHANNEL, &pwmInput);
      // If SRXL2 parsing fails, try next type
      if (pwmInput < PWM_INPUT_MIN || pwmInput > PWM_INPUT_MAX) inputType++; // Next input type
      break;
    }

    case PWM:
    {
      getPWMinput(currentTime, &pwmInput);
      if (pwmInput < PWM_INPUT_MIN || pwmInput > PWM_INPUT_MAX) pwmInput = 1000;  // No more input types to try....
      break;
    }

    default: // Should never happen
    {
      pwmInput = 1000;
      break;
    }
  }

  /************** LED output ***************/
  LED_Output(currentTime, pwmInput);
}
