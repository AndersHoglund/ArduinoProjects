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

#if defined(ARDUINO_BLUEPILL_F103C8)
#define DEBUG
#endif


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

#ifdef DEBUG
  Serial.begin(115200);
#endif

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
#ifdef DEBUG
      Serial.println("None");
#endif
      pwmInput = 1000;
      inputType++; // Next input type
      break;
    }

    case SRXL2:
    {
#ifdef DEBUGxx
      Serial.println("SRXL2");
#endif
      getSRXL2Pwm(currentTime, LED_CONTROL_CHANNEL, &pwmInput);
      // If SRXL2 parsing fails, try next type
      if (pwmInput < PWM_INPUT_MIN || pwmInput > PWM_INPUT_MAX) inputType++; // Next input type
      break;
    }

    case PWM:
    {
#ifdef DEBUGxx
      Serial.println("PWM");
#endif
      getPWMinput(currentTime, &pwmInput);
      if (pwmInput < PWM_INPUT_MIN || pwmInput > PWM_INPUT_MAX) pwmInput = 1000;  // No more input types to try....
      break;
    }

    default: // Should never happen
    {
      Serial.println("Ooops");
      pwmInput = 1000;
      break;
    }
  }

  /************** LED output ***************/
  LED_Output(currentTime, pwmInput);
}
