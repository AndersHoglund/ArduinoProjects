/*
  Fading

  This example shows how to fade an LED using the analogWrite() function.

  The circuit:
  - LED attached from digital pin 9 to ground.

  created 1 Nov 2008
  by David A. Mellis
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Fading
*/

int ledPin = 9;    // LED connected to digital pin 9

unsigned long currentTime;
unsigned long prevTime;
unsigned long fadePeriod = 30;
int dir = 0;
int fadeValue;

void setup() {
  // nothing happens in setup
}

void loop() 
{
  currentTime = millis();
  
  // fade in from min to max in increments of 5 points:
  if (currentTime - prevTime >= fadePeriod)
  {
    analogWrite(ledPin, fadeValue);

    prevTime = currentTime;

    if (dir == 0) fadeValue += 5;
    else fadeValue -=5;

    if (fadeValue >= 255) dir = 1;
    if (fadeValue == 0) dir = 0;
  }
}
