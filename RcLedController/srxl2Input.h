#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "spm_srxl.h"

#define SRXL2_PORT_BAUDRATE_DEFAULT 115200
#define SRXL2_FRAME_TIMEOUT 22

#if defined(ARDUINO_AVR_NANO)
#define srxl2port Serial


#elif defined(ARDUINO_GENERIC_STM32F103C)
#define srxl2port Serial1

#elif defined(ARDUINO_BLUEPILL_F103C8)
// place holder


#else
#error Bord type not supported
#endif

void setupSRXL2();
void getSRXL2Pwm(unsigned long currentTime, uint8_t rcChannel, uint16_t * pwmValuePtr);

#ifdef __cplusplus
} // extern "C"
#endif
