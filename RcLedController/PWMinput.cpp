/************* PWM Input driver *****************/

#include <Arduino.h>
#include "PWMinput.hpp"

static unsigned long prevPwmTime = 0;

void setupPWM(void)
{
  pinMode(INPUT_PIN, INPUT);
}

void getPWMinput(unsigned long currentTime, uint16_t * pwm)
{
  if (currentTime - prevPwmTime >= PWM_INTERVAL)
  {
    prevPwmTime = currentTime;
    *pwm = pulseInLong(INPUT_PIN, HIGH, 30000);
  }
}