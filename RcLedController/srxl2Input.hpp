#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "spm_srxl.h"

#define SRXL2_PORT_BAUDRATE_DEFAULT 115200
#define SRXL2_FRAME_TIMEOUT 22
#define srxl2port Serial

void setupSRXL2();
void getSRXL2Pwm(unsigned long currentTime, uint8_t rcChannel, uint16_t * pwmValuePtr);

#ifdef __cplusplus
} // extern "C"
#endif
