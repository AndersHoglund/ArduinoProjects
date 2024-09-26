/*
   Arduino 12 channel RC LED Controller

   Nav lights, ACL strobes, beacons and landing and reverse lights.
   Using SRXL2 or PWM input, automatically detedted.
   If you need SerialRx from a satellite Rx, or fixed single PWM input, checkout older versions from Git.
*/

#include "RcLedController_conf.h"

#include "srxl2Input.hpp"

#define INPUT_PIN 0           // RX0 pin, new wiring shared PWM/SRXL2 input pin
#define PWM_INPUT_MIN 800
#define PWM_INPUT_MAX 2200

// Classification.
#define NOT_USED         0
#define POWER_BEACON     1  // Always on when powered up.
#define SW_BEACON        2  // On/off controllable beacon.
#define BLINKER          3
#define SCOPE_TRIGGER    4

// PWM limits
#define LIGHTS_OFF    1000  // (Open Tx S1)
#define BEACON        1200     // NOTE: There can be only be fading beacons on pin 3, 5,6,9,10 and 11
#define POS_LIGHT     1300  // Turn on Red, Green nav light   (Open Tx S2)
#define ACL_STROBE    1400  // Turn on anti collition strobes (Open Tx S3)
#define LANDING_LIGHT 1500  // Turn on landing lights         (Open Tx S4)
#define BACKUP_LIGHT  1700  // Turn on reverse backup lights  (Open Tx S5)
#define ALL_ON        1900


typedef struct
{
  uint8_t  pin;
  uint8_t  steps;                // Number of blinks, strobes or fade-steps per period
  unsigned long period;     // Overall total period time for blinkers, fade step cycle time for faders.
  unsigned long duration;   // On and off time for multi blinkers, off time for faders.
  unsigned long prevTime;
  uint8_t  state;
  uint16_t pwm;
  uint16_t ledValue;
  uint8_t type;
} blinker_t;

blinker_t blinkers[] =
{
  { 2, 3, 1100,   40, 0, 0, ACL_STROBE,    0, BLINKER }, // Heli anti collition white tripple strobe
  { 3, 3, 1200,   40, 0, 0, ACL_STROBE,    0, BLINKER }, // Heli anti collition white tripple strobe
  { 4, 1, 1000, 1000, 0, 0, POS_LIGHT,     0, BLINKER }, // Red
  { 5, 3, 1300,   45, 0, 0, ACL_STROBE,    0, BLINKER }, // Heli anti collition white tripple strobe
  { 6, 1, 1000, 1000, 0, 0, POS_LIGHT,     0, BLINKER }, // Green
  { 7, 3, 1400,   50, 0, 0, ACL_STROBE,    0, BLINKER }, // Heli anti collition white tripple strobe
  { 8, 1, 1000, 1000, 0, 0, POS_LIGHT,     0, BLINKER }, // White
  { 9, 20,1200,   10, 0, 0, BEACON,        0, POWER_BEACON  }, // Red slow fading beacon. allways on when powered
  {10, 24, 800,   10, 0, 0, BEACON,        0, SW_BEACON  }, // Red belly fading beacon, slightly faster.
  {11, 1, 1000, 1000, 0, 0, LANDING_LIGHT, 0, BLINKER }, // White landing lights
  {12, 1, 1000, 1000, 0, 0, LANDING_LIGHT, 0, BLINKER }, // White landing lights
  {13, 1, 1000, 1000, 0, 0, BACKUP_LIGHT,  0, BLINKER }   // White backup/reverse lights
};

#define FIRST_LED_PIN blinkers[0].pin
#define LAST_LED_PIN  blinkers[sizeof(blinkers)/sizeof(blinker_t)-1].pin

// Ugly globals....
unsigned long currentTime;
uint16_t pwmInput = 1000;  // All lights off by default at power up

/**************************************************************/
void setup()
{
  setupSRXL2();

  pinMode(INPUT_PIN, INPUT);
  for (int i = 0; i < sizeof(blinkers) / sizeof(blinker_t); i++)
  {
    pinMode(blinkers[i].pin, OUTPUT);
  }
}

/**************************************************************/
void loop()
{
  static boolean usePwm = false;

/*********** SRXL2 input **************/
  currentTime = millis();
  if (!usePwm)
  {
    getSRXL2Pwm(currentTime, LED_CONTROL_CHANNEL, &pwmInput);
    // If SRXL2 parsing fails, switxh to PWM input
    if (pwmInput < PWM_INPUT_MIN || pwmInput > PWM_INPUT_MAX) usePwm = true;
  }

/************* PWM Input driver *****************/
  if (usePwm)
  {
    static unsigned long prevPwmTime = 0;
    const long pwmInterval = 1000;

    if (currentTime - prevPwmTime >= pwmInterval)
    {
      prevPwmTime = currentTime;
      pwmInput = pulseInLong(INPUT_PIN, HIGH, 30000);
      if (pwmInput < PWM_INPUT_MIN || pwmInput > PWM_INPUT_MAX) pwmInput = 1000;
    }
  }

/************** LED output driver ***************/

  for (int i = 0; i < sizeof(blinkers) / sizeof(blinker_t); i++ )
  {
    if ( blinkers[i].type == NOT_USED ||
         blinkers[i].pin   < FIRST_LED_PIN ||
         blinkers[i].pin   > LAST_LED_PIN )
    {
      continue; // Next
    }

    if (blinkers[i].type == POWER_BEACON)
    {
      fade(&blinkers[i]);
      continue; // Next
    }

    if (blinkers[i].type == SW_BEACON)
    {
      if (pwmInput > blinkers[i].pwm)
      {
        fade(&blinkers[i]);
      }
      continue; // Next
    }

    //if (blinkers[i].type == BLINKER)

    if (currentTime - blinkers[i].prevTime > blinkers[i].period - ((blinkers[i].steps - 1)*blinkers[i].duration * 2) )
    {
      blinkers[i].state = 0; // Start over
    }

    if (blinkers[i].state < (blinkers[i].steps * 2)) // Two states per blink, on and off.
    {
      if (currentTime - blinkers[i].prevTime > blinkers[i].duration )
      {
        togglePinState(&blinkers[i]);
      }
    }
  }
}

/**************************************************************/

void togglePinState(blinker_t * b)
{
  b->prevTime = currentTime;

  if ( (b->state & 0x01)  == 0 && pwmInput > b->pwm)
  {
    digitalWrite(b->pin, HIGH);
  }
  else
  {
    digitalWrite(b->pin, LOW);
  }
  b->state++;
}

/**************************************************************/
void fade(blinker_t * b)
{
  // fade in/out from min/max in increments of steps points:
  if ((b->state < 2) && (currentTime - b->prevTime >= b->duration))
  {
    if (b->state == 0)
    {
      b->ledValue += b->steps;
    }
    else if (b->state == 1)
    {
      b->ledValue -= b->steps;
    }

    analogWrite(b->pin, b->ledValue);
    b->prevTime = currentTime;

    if (b->ledValue >= 255)
    {
      b->state = 1;
    }
    if (b->ledValue <= 0)
    {
      b->state = 2;
    }
  }
  else if ( (b->state == 2) && (currentTime - b->prevTime >= b->period))
  {
    b->prevTime = currentTime;
    b->state = 0; //Start over.
  }
}
