
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#define FIRST_LED_PIN blinkers[0].pin
#define LAST_LED_PIN  blinkers[sizeof(blinkers)/sizeof(blinker_t)-1].pin

void setupLeds(void);
void LED_Output(unsigned long currentTime,uint16_t pwm);

#ifdef __cplusplus
} // extern "C"
#endif
