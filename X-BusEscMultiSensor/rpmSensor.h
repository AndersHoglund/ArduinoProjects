#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "spmTypes.h"

//must be an interrupt pin
#define PININ 3

//poles in motor
#define POLES 10

float getErpm();
float getRpm();

#ifdef __cplusplus
} // extern "C"
#endif
