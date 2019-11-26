/*
  Fading without using delay()

  This example shows how to fade an LED using the analogWrite() function.

  The circuit:
  - LED attached from digital pin 6 to ground.
  
*/

#define LED_PIN      6    // LED connected to digital pin 6
#define FADE_PERIOD 10
#define FADE_STEP 5;
  
unsigned long currentTime;

void setup() {
  // nothing happens in setup
}

void loop() 
{
  currentTime = millis();
  fade(LED_PIN);
}

void fade(int pin)
{
  static unsigned long ledFadeTime;
  static int dir = 0;
  static int fadeValue;

 // fade in/out from min/max in increments of 5 points:
  if (currentTime - ledFadeTime >= FADE_PERIOD)
  {
    analogWrite(pin, fadeValue);

    ledFadeTime = currentTime;

    if (dir == 0) {fadeValue += FADE_STEP;}
    else {fadeValue -= FADE_STEP;}

    if (fadeValue >= 255) { dir = 1;}
    if (fadeValue == 0)   { dir = 0;}
  }
}
