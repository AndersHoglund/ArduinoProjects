#include <Arduino.h>
#include <stdint.h>

#include "spmTypes.h"
#include "tempSensor.h"
#include "Thermistor.h"

UINT16 getTemp(UINT16 sensorPin)
 {
    return (getTemperature(sensorPin));
 }