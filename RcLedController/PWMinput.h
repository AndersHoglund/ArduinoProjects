#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(ARDUINO_AVR_NANO)
#define INPUT_PIN 0           // RX0 pin, new wiring shared PWM/SRXL2 input pin
  
#elif defined(ARDUINO_GENERIC_STM32F103C)
#define INPUT_PIN BOARD_USART1_RX_PIN

#elif defined(ARDUINO_BLUEPILL_F103C8) 
#define INPUT_PIN PA9         // UART 1 TX
#else
#error Bord type not supported
#endif

#define PWM_INPUT_MIN 800
#define PWM_INPUT_MAX 2200
#define PWM_INTERVAL 1000

void setupPWM(void);
void getPWMinput(unsigned long currentTime, uint16_t * pwm);

#ifdef __cplusplus
} // extern "C"
#endif
