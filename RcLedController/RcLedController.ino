/*
 * Asyncronuos multi blinks and fades
*/

#define PWM_INPUT_PIN 2
#define PWM_INPUT_MIN 800
#define PWM_INPUT_MAX 2200

// Classification and PWM types.
#define NOT_USED         0
#define BEACON           1  // Always on when powered up.
#define FADING_BEACON    2  // Always on when powered up. NOTE: There can be only be fading beacons on pin 3, 5,6,9,10 and 11
#define SCOPE_TRIGGER    3
#define POS_LIGHT     1200  // Turn on Red, Green and strobes
#define LANDING_LIGHT 1800  // Turn on landing lights

typedef struct 
{
  int pin;
  int steps;                // Number of blinks, strobes or fade-steps per period
  unsigned long period;     // Overall total period time for blinkers, fade step cycle time for faders.
  unsigned long duration;   // On and off time for multi blinkers, off time for faders.
  unsigned long prevTime;
  int state;
  double type;
  unsigned int ledValue;
} blinker_t;

blinker_t blinkers[] =
{
  { 3, 3, 1200,   40, 0, 0, POS_LIGHT,     0 }, // Heli anti collition white tripple strobe
  { 4, 1, 1000, 1000, 0, 0, POS_LIGHT,     0 }, // white
  { 5, 1, 1000, 1000, 0, 0, POS_LIGHT,     0 }, // Red
  { 6, 1, 1000, 1000, 0, 0, POS_LIGHT,     0 }, // Green
  { 7, 1, 1000, 1000, 0, 0, LANDING_LIGHT, 0 }, // White landding lights
  { 8, 1, 1000, 1000, 0, 0, LANDING_LIGHT, 0 }, // White landding lights
  { 9,10,    5,  300, 0, 0, FADING_BEACON, 0 }, // Heli tail red slow fading beacon.
  {10,12,    5,  200, 0, 0, FADING_BEACON, 0 }, // Red belly fading beacon, slightly faster.
  {11, 3, 1500,   50, 0, 0, POS_LIGHT,     0 }, // Heli anti collition white tripple strobe
  {12, 0,    0,    0, 0, 0, NOT_USED,      0 }, // Not used, yet...
  {13, 1, 2500,   10, 0, 0, SCOPE_TRIGGER, 0}  //
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
      continue; // Next
    }
    
    if (blinkers[i].type == FADING_BEACON)
    {
      fade(&blinkers[i]);
      continue; // Next
    }

    if (currentTime - blinkers[i].prevTime > blinkers[i].period - ((blinkers[i].steps -1)*blinkers[i].duration*2) )
    {
      blinkers[i].state = 0; // Start over
    }

    if (blinkers[i].state < (blinkers[i].steps * 2)) // Two states per blink, on and off.
    {
      if (currentTime - blinkers[i].prevTime >= blinkers[i].duration )
      {
        togglePinState(&blinkers[i]);
      }
    }
  }
}

/**************************************************************/

void togglePinState(blinker_t * b)
{
  b->prevTime = currentTime;

  if ( (b->state & 0x01)  == 0 && pwmInput > b->type)
  {
    digitalWrite(b->pin, HIGH);
  }
  else
  {
    digitalWrite(b->pin, LOW);
  }
  b->state++;
}

/**************************************************************/
void fade(blinker_t * b)
{

 // fade in/out from min/max in increments of steps points:
  if ((b->state < 2) && (currentTime - b->prevTime >= b->period))
  {
    if      (b->state == 0)   {b->ledValue += b->steps;}
    else if (b->state == 1)   {b->ledValue -= b->steps;}

    analogWrite(b->pin, b->ledValue);
    b->prevTime = currentTime;

    if (b->ledValue >= 255) { b->state = 1;}
    if (b->ledValue <= 0)   { b->state = 2;}
  }
  else if ( (b->state == 2) && (currentTime - b->prevTime >= b->duration))
  {
    b->prevTime = currentTime;
    b->state = 0; //Start over.
  }
}
