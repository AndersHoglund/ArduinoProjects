
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// LED to pin connections
#if defined(ARDUINO_AVR_NANO)

#define LED_1 2
#define LED_2 3
#define LED_3 4
#define LED_4 5
#define LED_5 6
#define LED_6 7
#define LED_7 8
#define LED_8 9
#define LED_9 10
#define LED_10 11
#define LED_11 12
#define LED_12 13

#define LED_ON HIGH
#define LED_OFF LOW

#elif defined(ARDUINO_GENERIC_STM32F103C) || defined(ARDUINO_BLUEPILL_F103C8)

#define LED_1 PB12
#define LED_2 PB13
#define LED_3 PB14
#define LED_4 PB15
#define LED_5 PB5
#define LED_6 PB6
#define LED_7 PB7
#define LED_8 PB8
#define LED_9 PB9
#define LED_10 PB10
#define LED_11 PB11
#define LED_12 PC13

#define LED_ON LOW
#define LED_OFF HIGH

#else
#error Bord type not supported
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
