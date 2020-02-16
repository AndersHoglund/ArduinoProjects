/*
   Asyncronuos multi blinks and fades
*/
#define USE_SERIAL_RX_INPUT
//#define USE_PWM_INPUT

#ifdef USE_SERIAL_RX_INPUT
#include <SoftwareSerial.h>

// Spektrum channel order
#define AILE 0
#define ELEV 1
#define THRO 2
#define ROLL 3
#define GEAR 4
#define AUX1 5
#define AUX2 6
#define AUX3 7
#define AUX4 8
#define AUX5 9

// Only available at 22ms frame rate, not at 11ms.
#define AUX6 10
#define AUX7 11

// Select what channel to use
#define LED_CONTROL_CHANNEL AUX5

// Select Arduino input pin
#define INPUT_PIN 2

// Only support DSMx SERIALRX_SPEKTRUM2048:
#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEK_FRAME_SIZE                      16
#define SPEK_CHAN_SHIFT                       3
#define SPEK_CHAN_MASK                     0x07
#define SPEKTRUM_NEEDED_FRAME_INTERVAL        5
#define SPEKTRUM_BAUDRATE                115200

SoftwareSerial serialRx(INPUT_PIN, 15); // RX, TX
#endif

#ifdef USE_PWM_INPUT
#define INPUT_PIN 2
#define PWM_INPUT_MIN 800
#define PWM_INPUT_MAX 2200
#endif

// Classification and PWM types.
#define NOT_USED         0
#define BEACON           1  // Always on when powered up.
#define FADING_BEACON    2  // Always on when powered up. NOTE: There can be only be fading beacons on pin 3, 5,6,9,10 and 11
#define SCOPE_TRIGGER    3
#define POS_LIGHT     1400  // Turn on Red, Green and strobes
#define LANDING_LIGHT 1700  // Turn on landing lights

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
  { 4, 1, 1000, 1000, 0, 0, POS_LIGHT,     0 }, // Red
  { 5, 3, 1300,   45, 0, 0, POS_LIGHT,     0 }, // Heli anti collition white tripple strobe
  { 6, 1, 1000, 1000, 0, 0, POS_LIGHT,     0 }, // Green
  { 7, 3, 1400,   50, 0, 0, POS_LIGHT,     0 }, // Heli anti collition white tripple strobe
  { 8, 1, 1000, 1000, 0, 0, POS_LIGHT,     0 }, // White
  { 9, 20,1200,   10, 0, 0, FADING_BEACON, 0 }, // Heli tail red slow fading beacon.
  {10, 24, 800,   10, 0, 0, FADING_BEACON, 0 }, // Red belly fading beacon, slightly faster.
  {11, 1, 1000, 1000, 0, 0, LANDING_LIGHT, 0 }, // White landding lights
  {12, 1, 1000, 1000, 0, 0, LANDING_LIGHT, 0 }, // White landding lights
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
  for (int i = 0; i < sizeof(blinkers) / sizeof(blinker_t); i++)
  {
    pinMode(blinkers[i].pin, OUTPUT);
  }

  pinMode(INPUT_PIN, INPUT);

#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("Goodnight moon!");
#endif

#ifdef USE_SERIAL_RX_INPUT
  serialRx.begin(SPEKTRUM_BAUDRATE);
#endif

}

/**************************************************************/
void loop()
{

  currentTime = millis();

#ifdef USE_SERIAL_RX_INPUT
  static unsigned long prevSerialRxTime = 0;
  static unsigned char spekFrame[SPEK_FRAME_SIZE];
  static unsigned char spekFramePosition = 0;
  static bool rcFrameComplete = false;

  unsigned long spekChannelData[SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT];

  if (currentTime - prevSerialRxTime > SPEKTRUM_NEEDED_FRAME_INTERVAL)
  {
    prevSerialRxTime = currentTime;
    spekFramePosition = 0;
  }

  if (spekFramePosition < SPEK_FRAME_SIZE && serialRx.available())
  {
    unsigned char c = serialRx.read();
    spekFrame[spekFramePosition++] = c;
    
    if (spekFramePosition < SPEK_FRAME_SIZE)
    {
      rcFrameComplete = false;
    }
    else
    {
      rcFrameComplete = true;
    }
  }

  if (rcFrameComplete)
  {
    rcFrameComplete = false;

    // Get the RC control channel inputs
    for (int b = 3; b < SPEK_FRAME_SIZE; b += 2)
    {
      const uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> SPEK_CHAN_SHIFT);

      if (spekChannel < SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT)
      {
        spekChannelData[spekChannel] = ((spekFrame[b - 1] & SPEK_CHAN_MASK) << 8) + spekFrame[b];
      }
    }

    // Convert to PWM 1000 .. 1500 .. 2000 value range
    pwmInput = 988 + (spekChannelData[LED_CONTROL_CHANNEL] >> 1);   // 2048 mode
  }
#endif

#ifdef USE_PWM_INPUT
  static unsigned long prevPwmTime = 0;
  const long pwmInterval = 1000;

  if (currentTime - prevPwmTime >= pwmInterval)
  {
    prevPwmTime = currentTime;
    pwmInput = pulseInLong(INPUT_PIN, HIGH, 30000);
    if (pwmInput < PWM_INPUT_MIN || pwmInput > PWM_INPUT_MAX) pwmInput = 1000;
  }
#endif

  for (int i = 0; i < sizeof(blinkers) / sizeof(blinker_t); i++ )
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

    if (currentTime - blinkers[i].prevTime > blinkers[i].period - ((blinkers[i].steps - 1)*blinkers[i].duration * 2) )
    {
      blinkers[i].state = 0; // Start over
    }

    if (blinkers[i].state < (blinkers[i].steps * 2)) // Two states per blink, on and off.
    {
      if (currentTime - blinkers[i].prevTime > blinkers[i].duration )
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
  if ((b->state < 2) && (currentTime - b->prevTime >= b->duration))
  {
    if (b->state == 0)
    {
      b->ledValue += b->steps;
    }
    else if (b->state == 1)
    {
      b->ledValue -= b->steps;
    }

    analogWrite(b->pin, b->ledValue);
    b->prevTime = currentTime;

    if (b->ledValue >= 255)
    {
      b->state = 1;
    }
    if (b->ledValue <= 0)
    {
      b->state = 2;
    }
  }
  else if ( (b->state == 2) && (currentTime - b->prevTime >= b->period))
  {
    b->prevTime = currentTime;
    b->state = 0; //Start over.
  }
}
