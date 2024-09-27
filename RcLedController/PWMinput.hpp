#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INPUT_PIN 0           // RX0 pin, new wiring shared PWM/SRXL2 input pin
#define PWM_INPUT_MIN 800
#define PWM_INPUT_MAX 2200
#define PWM_INTERVAL 1000

void setupPWM(void);
void getPWMinput(unsigned long currentTime, uint16_t * pwm);

#ifdef __cplusplus
} // extern "C"
#endif