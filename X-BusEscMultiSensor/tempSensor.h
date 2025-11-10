#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "spmTypes.h"

// select the analog voltage input pin
#define FET_SENSOR_PIN A7
#define BEC_SENSOR_PIN A6

UINT16 getTemp(UINT16 sensorPin);

#ifdef __cplusplus
} // extern "C"
#endif