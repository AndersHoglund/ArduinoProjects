/*
 * Asyncronuos multi blinks
*/

#define PWM_INPUT_PIN 2
#define PWM_INPUT_MIN 800
#define PWM_INPUT_MAX 2200

// Classification and PWM types.
#define NOT_USED         0
#define BEACON           1  // Always on when powered up. 
#define FADING_BEACON    2  // Always on when powered up. NOTE: There can be only one fading beacon and only on pin 3, 5,6or 9
#define SCOPE_TRIGGER    3
#define POS_LIGHT     1200  // Turn on Red, Green and strobes 
#define LANDING_LIGHT 1800  // Turn on landing lifghts

typedef struct 
{
  int pin;
  int numBlinks;            // Number of blinks or strobes per period
  unsigned long period;     // Overall total period time
  unsigned long duration;   // On time 
  unsigned long prevTime;
  int state;
  double type;
} blinker_t;

blinker_t blinkers[] = 
{
  { 3, 3, 1200,   40, 0, 0, POS_LIGHT },    // Heli anti collition white tripple strobe
  { 4, 1, 1000, 1000, 0, 0, POS_LIGHT },    // white
  { 5, 1, 1000, 1000, 0, 0, POS_LIGHT },    // Red
  { 6, 1, 1000, 1000, 0, 0, POS_LIGHT },    // Red
  { 7, 1, 1000, 1000, 0, 0, POS_LIGHT },    // Green
  { 8, 1, 1000, 1000, 0, 0, LANDING_LIGHT },// White landding lights
  { 9, 8,   10,    0, 0, 0, FADING_BEACON },// Heli tail red beacon. NOTE: There can be only one fading beacon
  {10, 2, 2100,   90, 0, 0, BEACON },       // Red belly blinker beacon
  {11, 3, 1500,   50, 0, 0, POS_LIGHT },    // Heli anti collition white tripple strobe
  {12, 0,    0,    0, 0, 0, NOT_USED },     // Not used, yet... 
  {13, 1, 2500,   10, 0, 0, SCOPE_TRIGGER}  // 
};

#define FIRST_LED_PIN 3
#define LAST_LED_PIN 13

// Ugly globals....
unsigned long currentTime;
double pwmInput;

/**************************************************************/
void setup() 
{
  for (int i = 0; i < sizeof(blinkers)/sizeof(blinker_t); i++)
  {
    pinMode(blinkers[i].pin, OUTPUT);
  }
  
  pinMode(PWM_INPUT_PIN, INPUT);

}

/**************************************************************/
void loop() 
{

  currentTime = millis();
  unsigned long prevPwmTime = 0;
  const long pwmInterval = 1000;

  if (currentTime - prevPwmTime >= pwmInterval)
  {
    prevPwmTime = currentTime;
    pwmInput = pulseInLong(PWM_INPUT_PIN, HIGH, 30000);
    if (pwmInput < PWM_INPUT_MIN || pwmInput > PWM_INPUT_MAX) pwmInput = 1000; 
  }

  for (int i = 0; i < sizeof(blinkers)/sizeof(blinker_t); i++ )
  {
    if ( blinkers[i].type == NOT_USED ||
         blinkers[i].pin   < FIRST_LED_PIN || 
         blinkers[i].pin   > LAST_LED_PIN )
    { 
        i++; // Next
    }
    
    if (blinkers[i].type == FADING_BEACON)
    {
      blinkers[i].prevTime = fade(blinkers[i].pin, blinkers[i].numBlinks, blinkers[i].period, blinkers[i].prevTime);
    }

    if (currentTime - blinkers[i].prevTime > blinkers[i].period - ((blinkers[i].numBlinks -1)*blinkers[i].duration*2) )
    {
      blinkers[i].state = 0; // Start over
    }

    if (blinkers[i].state < (blinkers[i].numBlinks * 2)) // Two states per blink, on and off.
    {
      if (currentTime - blinkers[i].prevTime >= blinkers[i].duration ) 
      {
        blinkers[i].prevTime = currentTime;
        blinkers[i].state = togglePinState(blinkers[i].pin, blinkers[i].state, blinkers[i].type);
      }
    }
  }
}

/**************************************************************/

int togglePinState(int pin, int state, double type)
{
  int newState;
    if ( (state & 0x01)  == 0 && pwmInput > type)  
    {
      newState = HIGH;
    } 
    else 
    {
      newState = LOW;
    }

    digitalWrite(pin, newState);

    state++;
    return state;
}

/**************************************************************/

unsigned long fade(int pin, int fadeStep, unsigned long period, unsigned long ledFadeTime)
{
  static int dir = 0;
  static int fadeValue = 0;

 // fade in/out from min/max in increments of 5 points:
  if (currentTime - ledFadeTime >= period)
  {
    analogWrite(pin, fadeValue);

    ledFadeTime = currentTime;

    if (dir == 0) {fadeValue += fadeStep;}
    else {fadeValue -= fadeStep;}

    if (fadeValue >= 255) { dir = 1;}
    if (fadeValue == 0)   { dir = 0;}
  }
  return ledFadeTime;
}
