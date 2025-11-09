// Copied and modified Thermistor Example #3 from the Adafruit Learning System guide on Thermistors 
// https://learn.adafruit.com/thermistor/overview by Limor Fried, Adafruit Industries
// MIT License - please keep attribution and consider buying parts from Adafruit

#include <Arduino.h>
#include <stdint.h>

#include "Thermistor.h"

int samples[NUMSAMPLES];

int getTemperature(int analogPin) 
{
  unsigned char i;
  float average;

  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(analogPin);
   delay(10);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;

#ifdef DEBUG
  Serial.print("Average analog reading "); 
  Serial.println(average);
#endif

  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

#ifdef DEBUG
  Serial.print("Thermistor resistance "); 
  Serial.println(average);
#endif
  
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C
  
#ifdef DEBUG
  Serial.print("Temperature "); 
  Serial.print(steinhart);
  Serial.println(" *C");
#endif

  int centiGrade = steinhart; 
  return(centiGrade);
}
