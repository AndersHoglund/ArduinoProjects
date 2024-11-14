/************** LED output driver ***************/

#include <Arduino.h>
#include "LedOutput.h"

blinker_t blinkers[] =
{
  { LED_1,  3, 1100,   40, 0, 0, ACL_STROBE,    0, BLINKER }, // Heli anti collition white tripple strobe
  { LED_2,  3, 1200,   40, 0, 0, ACL_STROBE,    0, BLINKER }, // Heli anti collition white tripple strobe
  { LED_3,  1, 1000, 1000, 0, 0, POS_LIGHT,     0, BLINKER }, // Red
  { LED_4,  3, 1300,   45, 0, 0, ACL_STROBE,    0, BLINKER }, // Heli anti collition white tripple strobe
  { LED_5,  1, 1000, 1000, 0, 0, POS_LIGHT,     0, BLINKER }, // Green
  { LED_6,  3, 1400,   50, 0, 0, ACL_STROBE,    0, BLINKER }, // Heli anti collition white tripple strobe
  { LED_7,  1, 1000, 1000, 0, 0, POS_LIGHT,     0, BLINKER }, // White
  { LED_8, 20, 1200,   10, 0, 0, BEACON,        0, POWER_BEACON  }, // Red slow fading beacon. allways on when powered
  { LED_9, 24,  800,   10, 0, 0, BEACON,        0, SW_BEACON  }, // Red belly fading beacon, slightly faster.
  { LED_10, 1, 1000, 1000, 0, 0, LANDING_LIGHT, 0, BLINKER }, // White landing lights
  { LED_11, 1, 1000, 1000, 0, 0, LANDING_LIGHT, 0, BLINKER }, // White landing lights
  { LED_12, 1, 1000, 1000, 0, 0, BACKUP_LIGHT,  0, BLINKER }   // White backup/reverse lights
};

// Fwd declarations
void fade(blinker_t * b);
void togglePinState(blinker_t * b, uint16_t pwm);

static unsigned long currentTime;

void setupLeds()
{
  for (int i = 0; i < sizeof(blinkers) / sizeof(blinker_t); i++)
  {
    pinMode(blinkers[i].pin, OUTPUT);
  }
}

void LED_Output(unsigned long ct, uint16_t pwm)
{
  currentTime = ct;
  for (int i = 0; i < sizeof(blinkers) / sizeof(blinker_t); i++ )
  {
    if ( blinkers[i].type == NOT_USED )
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
      if (pwm > blinkers[i].pwm)
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
        togglePinState(&blinkers[i], pwm);
      }
    }
  }
}

/**************************************************************/

void togglePinState(blinker_t * b, uint16_t pwm)
{
  b->prevTime = currentTime;

  if ( (b->state & 0x01)  == 0 && pwm > b->pwm)
  {
    digitalWrite(b->pin, LED_ON);
  }
  else
  {
    digitalWrite(b->pin, LED_OFF);
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
