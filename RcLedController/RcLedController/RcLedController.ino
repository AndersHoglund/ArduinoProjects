/*
 * Asyncronuos multi blinks
*/

#define PWM_INPUT 2

// Classification and PWM conditions.
#define BEACON           0  // Always on when poered up. 
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
  double condition;
} blinker_t;

blinker_t blinkers[] = 
{
  { 4, 1, 1000, 1000, 0, 0, POS_LIGHT },    // Red
  { 5, 1, 1000, 1000, 0, 0, POS_LIGHT },    // Green
  { 6, 2, 1000,  100, 0, 0, BEACON },       // Heli tail red beacon
  { 7, 1, 1000, 1000, 0, 0, LANDING_LIGHT },// White landding lights
  { 8, 2, 2100,   90, 0, 0, BEACON },       // Red belly beacon
  { 9, 3, 1500,   50, 0, 0, POS_LIGHT }     // Heli anti collition white tripple strobe
};

double pwmInput;

/**************************************************************/
void setup() 
{
  for (int i = 0; i < sizeof(blinkers)/sizeof(blinker_t); i++)
  {
    pinMode(blinkers[i].pin, OUTPUT);
  }
  
  pinMode(PWM_INPUT, INPUT);

}

/**************************************************************/
void loop() 
{

  unsigned long currentTime = millis();
  unsigned long prevPwmTime = 0;
  const long pwmInterval = 1000;

  if (currentTime - prevPwmTime >= pwmInterval)
  {
    prevPwmTime = currentTime;
    pwmInput = pulseInLong(PWM_INPUT, HIGH, 30000);
  }

  for (int i = 0; i < sizeof(blinkers)/sizeof(blinker_t); i++ )
  {
    if (currentTime - blinkers[i].prevTime > blinkers[i].period - ((blinkers[i].numBlinks -1)*blinkers[i].duration*2) )
    {
      blinkers[i].state = 0; // Start over
    }

    if (blinkers[i].state < (blinkers[i].numBlinks * 2)) // Two states per blink, on and off.
    {
      if (currentTime - blinkers[i].prevTime >= blinkers[i].duration ) 
      {
        blinkers[i].prevTime = currentTime;
        blinkers[i].state = togglePinState(blinkers[i].pin, blinkers[i].state, blinkers[i].condition);
      }
    }
  }
}

/**************************************************************/

int togglePinState(int pin, int state, double condition)
{
  int newState;
    if ( (state & 0x01)  == 0 && pwmInput > condition)  
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
