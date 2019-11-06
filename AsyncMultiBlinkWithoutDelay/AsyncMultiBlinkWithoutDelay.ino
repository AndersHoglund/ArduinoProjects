/*
 * Asyncronuos multi blinks
*/

typedef struct {
  int pin;
  int numBlinks;            // Number of blinks or strobes per period
  unsigned long period;     // Overall total period time
  unsigned long duration;   // On time 
  unsigned long prevTime;
  int state;
} blinker_t;

blinker_t blinkers[] = {
  { 4, 1, 900, 450, 0, 0 },    // Normal 50/50 blink
  { 5, 1, 800, 400, 0, 0 },
  { 6, 2, 1000,100, 0, 0 },    // Heli tail beacon
  { 7, 2, 2000,100, 0, 0 },    // Anti collition double strobe
  { 8, 1, 500, 250, 0, 0 },    
  { 9, 3, 1500, 50, 0, 0 }     // Heli anti collition tripple strobe
};

/**************************************************************/
void setup() 
{
  for (int i = 0; i < sizeof(blinkers)/sizeof(blinker_t); i++)
  {
    pinMode(blinkers[i].pin, OUTPUT);
  }
}

/**************************************************************/
void loop() 
{

  unsigned long currentTime = millis();

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
        blinkers[i].state = togglePinState(blinkers[i].pin, blinkers[i].state, blinkers[i].numBlinks);
      }
    }
  }
}

/**************************************************************/

int togglePinState(int pin, int state, int num)
{
  int newState;
    if ( (state & 0x01)  == 0)  
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
