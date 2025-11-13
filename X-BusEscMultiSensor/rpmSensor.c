#include <Arduino.h>
#include <stdint.h>

#include "spmTypes.h"
#include "rpmSensor.h"

volatile byte pulses;
static float rpm;
static float eRpm;
static unsigned long timeStamp;
static unsigned long newTime;

//if there's a new timestamp
bool newTimeStamp;

//sample threshold. arbitrary
#define THRESH 20

//milliseconds in a minute
#define MILTOMIN 60000.0

static boolean initialized = false;

void revCount()
{
  //update pulse
   pulses++;
   if (pulses >= THRESH) {
       newTime = millis();
       newTimeStamp = true; // flag to tell main code to read the value of timeStamp

       digitalWrite(13,HIGH);           // Debug LED and scope trigger
  }
}

void setupRpmMeter()
{
 pulses = 0;
 rpm = 0;
 eRpm = 0;
 timeStamp = 0;
 newTimeStamp = true;
 attachInterrupt(digitalPinToInterrupt(PININ), revCount, FALLING);
 initialized = true;
}

float getErpm()
{
 if (!initialized) {setupRpmMeter();}

 //have THRESH samples been taken?
 if (newTimeStamp)
  {
    noInterrupts();//disable interrupt
    newTimeStamp = false;//toggle newTimeStamp back
    eRpm = ((float)pulses / ( float)(newTime-timeStamp)) * MILTOMIN ;

    //reset pulse and timestamp
    pulses = 0;
    timeStamp = millis();
    interrupts(); //reattach interrupt

    digitalWrite(13, LOW);
  }
  return (eRpm);
}

float getRpm()
{
  getErpm();
  rpm = eRpm/(POLES/2);
  return(rpm);
}

