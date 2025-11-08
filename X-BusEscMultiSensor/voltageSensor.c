#include <Arduino.h>
#include <stdint.h>

#include "spmTypes.h"
#include "voltageSensor.h"

// select the analog voltage input pin
#define SENSOR_PIN A0    

// Define voltage divider resistor values
#define RESISTOR_HIGH (330.0) // kOhms
#define RESISTOR_LOW (33.0)

#define ADC_SCALE 5.0/1023.0
#define SCALE ((double)((RESISTOR_LOW + RESISTOR_HIGH) / RESISTOR_LOW) * ADC_SCALE)

double getVoltage()
 {
    // read the value from the ADC:  
    UINT16 sensorValue = analogRead(SENSOR_PIN);
  
    // compute real voltage
    return (double)((sensorValue) * SCALE);
 }
