//////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SRXL2 decoder example for Base Receivers and ATMega328P. Used receiver in this exampel is the AR6610T.
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////

#include <avr/interrupt.h>
#include <avr/wdt.h>//watchdog
#include "spm_srxl.h"

#if defined(ARDUINO_AVR_MICRO)
// AT32U4
#define TIMERX TIMER3
#define TCNTX  TCNT3
#define TCCRXA TCCR3A
#define TCCRXB TCCR3B
#define TIFRX  TIFR3
#define TIMSKX TIMSK3
#define OCRXA  OCR3A
#define OCRXB  OCR3B
#elif defined (ARDUINO_AVR_NANO) or defined (ARDUINO_AVR_MINI) or defined (ARDUINO_AVR_PRO)
// AT32328P 16Mhz really
#define TIMERX TIMER2
#define TCNTX  TCNT2
#define TCCRXA TCCR2A
#define TCCRXB TCCR2B
#define TIFRX  TIFR2
#define TIMSKX TIMSK2
#define OCRXA  OCR2A
#define OCRXB  OCR2B
#else
#error "Board type not supported"
#endif

#define SRXL2_PORT_BAUDRATE_DEFAULT 115200
#define SRXL2_FRAME_TIMEOUT 22
#define srxl2port Serial
#define NO_OF_INPUT_CHANNELS 18
#define NO_OF_OUTPUT_CHANNELS 18

#define MAX_NO_OF_CHANNELS 20

#if (NO_OF_OUTPUT_CHANNELS > MAX_NO_OF_CHANNELS)
error
#endif

// Spektrum channel order
#define THRO 0
#define AILE 1
#define ELEV 2
#define YAW  3
#define GEAR 4
#define AUX1 5
#define AUX2 6
#define AUX3 7
#define AUX4 8
#define AUX5 9

// Only available at 22ms frame rate.
#define AUX6 10
#define AUX7 11

// X-Plus channels, non DX18 mode.
#define X1 12
#define X2 13
#define X3 14
#define X4 15
#define X5 16
#define X6 17
#define X7 18
#define X8 19

#define DSMX_2K_LOW     341  // -100%
#define DSMX_2K_CENTER 1024  //    0%
#define DSMX_2K_HIGH   1706  // +100%
#define DSMX_2K_OFFSET 8991
 
#define YES 1
#define NO 0
#define UseJitterCompensation YES

unsigned long currentTime;
unsigned long prevPwmTime = 0;
const long pwmInterval = 22;

boolean validServoPacket   = false;
boolean timersRunning = false;

// Array to store the input PWM servo position
uint16_t pwmPos[NO_OF_INPUT_CHANNELS];

// Chanel mapping to outputs                      D2    D3    D4    D5    D6    D7    D8    D9    D10   D11  D12 D13  A0  A1  A2  A3  A4  A5
uint16_t inputChannelMap[NO_OF_INPUT_CHANNELS] = {THRO, ELEV, GEAR, AUX1, AUX2, AUX3, AUX4, AUX5, AUX6, AUX7, X1, X2, X3, X4, X5, X6, X7, X8};

static byte Jitter;
static byte Jitter2;
static byte Jitter3;
static byte Jitter4;
//static byte RealTime5s;
static unsigned int iCount;
static volatile uint8_t *OutPortTable[MAX_NO_OF_CHANNELS] = {&PORTD,&PORTD,&PORTD,&PORTD,&PORTD,&PORTD,&PORTB,&PORTB,&PORTB,&PORTB,&PORTB,&PORTB,&PORTC,&PORTC,&PORTC,&PORTC,&PORTC,&PORTC,&PORTC,&PORTC};
static uint8_t OutBitTable[MAX_NO_OF_CHANNELS] = {4,8,16,32,64,128,1,2,4,8,16,32,1,2,4,8,16,32,64,128};
static unsigned int ServoPW[MAX_NO_OF_CHANNELS];
static byte TIMERXToggle;
static volatile uint8_t *OutPort1A = &PORTD;
static volatile uint8_t *OutPort1B = &PORTB;
static uint8_t OutBit1A = 0;
static uint8_t OutBit1B = 0;
static volatile uint8_t *OutPortNext1A = &PORTD;
static volatile uint8_t *OutPortNext1B = &PORTB;
static uint8_t OutBitNext1A = 0;
static uint8_t OutBitNext1B = 0;

void setup()
{
  initServoOutputs();
  defaultServoValues();

  srxl2port.begin(SRXL2_PORT_BAUDRATE_DEFAULT);
  srxlInitDevice(SRXL_DEVICE_ID, SRXL_DEVICE_PRIORITY, SRXL_DEVICE_INFO, 0x01000001);
  srxlInitBus(0, 1, SRXL_SUPPORTED_BAUD_RATES);

  wdt_enable(WDTO_250MS);   // Set watchdog to 0.25 s and start.
}

void loop() {
  currentTime = millis();

  static unsigned long prevSerialRxTime = 0;

  // UART receive buffer
  static uint8_t rxBuffer[2 * SRXL_MAX_BUFFER_SIZE];
  static uint8_t rxBufferIndex = 0;

  if (currentTime - prevSerialRxTime > SRXL2_FRAME_TIMEOUT)
  {
    prevSerialRxTime = currentTime;
    rxBufferIndex = 0;
    srxlRun(0, SRXL2_FRAME_TIMEOUT);
  }

  if ( srxl2port.available() )
  {
    prevSerialRxTime = currentTime;
    unsigned char c = srxl2port.read(); // 
    rxBuffer[rxBufferIndex++] = c;
  }

  if (rxBufferIndex >= 5)
  {
    if(rxBuffer[0] == SPEKTRUM_SRXL_ID)
    {
      uint8_t packetLength = rxBuffer[2];
      if (rxBufferIndex >= packetLength)
      {
        // Try to parse SRXL packet -- this internally calls srxlRun() after packet is parsed and reset timeout
        if (srxlParsePacket(0, rxBuffer, packetLength))
        {
          // Move any remaining bytes to beginning of buffer (usually 0)
          rxBufferIndex -= packetLength;
          memmove(rxBuffer, &rxBuffer[packetLength], rxBufferIndex);
        }
        else
        {
            rxBufferIndex = 0;
        }
      }
    }
  }

  if (currentTime - prevPwmTime >= pwmInterval)
  {
    prevPwmTime = currentTime;
    for (int i=0; i < NO_OF_INPUT_CHANNELS; i++)
    {
      ServoPW[i] = pwmPos[i];
    }
    PPM();                                  // Do servo move
  }
}

volatile void PPM() //Move servos every 22ms to the desired position.
{
  wdt_reset();// Watchdog zurücksetzen. Reset the Watchdog.
  if (validServoPacket && (!timersRunning)) ServoSetup();
}

ISR(TIMER1_COMPA_vect) // Interrupt routine for timer 1 compare A. Used for timing each pulse width for the servo PWM.
{ 
#if UseJitterCompensation == YES
    Jitter = TCNT1 - OCR1A;
    if(Jitter == 32){asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");}
    if(Jitter == 31){asm volatile("nop\n\tnop\n\tnop\n\t");}
    if(Jitter == 30){asm volatile("nop\n\t");}
    if(Jitter == 29){asm volatile("nop\n\t");}
#endif
  *OutPort1A &= ~OutBit1A;                //Pulse A finished. Set to low
}

ISR(TIMER1_COMPB_vect) // Interrupt routine for timer 1 compare B. Used for timing each pulse width for the servo PWM.
{ 
#if UseJitterCompensation == YES
    Jitter2 = TCNT1 - OCR1B;
    if(Jitter2 == 32){asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");}
    if(Jitter2 == 31){asm volatile("nop\n\tnop\n\tnop\n\t");}
    if(Jitter2 == 30){asm volatile("nop\n\t");}
    if(Jitter2 == 29){asm volatile("nop\n\t");}
#endif
  *OutPort1B &= ~OutBit1B;                //Pulse B finished. Set to low
}

ISR(TIMERX_COMPA_vect) // Interrupt routine for timer 2 compare A. Used for timing 50Hz for each servo.
{ 
#if UseJitterCompensation == YES
    Jitter4 = TCNT1L-100;
    if(Jitter4 == 118){asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");}
    if(Jitter4 == 117){asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");}
    if(Jitter4 == 116){asm volatile("nop\n\tnop\n\tnop\n\t");}
    if(Jitter4 == 115){asm volatile("nop\n\t");}
    if(Jitter4 == 114){asm volatile("nop\n\t");}
#endif
  *OutPortNext1A |= OutBitNext1A;         // Start new pulse on next servo. Write pin HIGH
  *OutPortNext1B |= OutBitNext1B;         // Start new pulse on next servo. Write pin HIGH
}

ISR(TIMERX_COMPB_vect) // Interrupt routine for timer 3 compare B. Used for timing 50Hz for each servo.
{ 
  TIFR1 = 255;                                       // Clear  pending interrupts
#if UseJitterCompensation == YES
    Jitter3 = TCNT1L-100;
    if(Jitter3 == 137){asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");}
    if(Jitter3 == 136){asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");}
    if(Jitter3 == 135){asm volatile("nop\n\tnop\n\tnop\n\t");}
    if(Jitter3 == 134){asm volatile("nop\n\t");}
    if(Jitter3 == 133){asm volatile("nop\n\t");}
#endif
  TCNT1 = 0;                                         // Restart counter for timer1
  TCNTX = 0;                                         // Restart counter for timer2
  sei();
  *OutPort1A &= ~OutBit1A;                           // Set pulse low to if not done already
  *OutPort1B &= ~OutBit1B;                           // Set pulse low to if not done already
  OutPort1A = OutPortTable[TIMERXToggle];            // Temp port for COMP1A
  OutBit1A = OutBitTable[TIMERXToggle];              // Temp bitmask for COMP1A
  OutPort1B = OutPortTable[TIMERXToggle+10];         // Temp port for COMP1B
  OutBit1B = OutBitTable[TIMERXToggle+10];           // Temp bitmask for COMP1B

  OCR1A = ServoPW[TIMERXToggle]-8020;
  OCR1B = ServoPW[TIMERXToggle+10]-8015;
  TIMERXToggle++;                                    // Next servo in line.
  if(TIMERXToggle==10)
  { 
    TIMERXToggle = 0;                                // If next servo is grater than 9, start on 0 again.
                                   
  }
  OutPortNext1A = OutPortTable[TIMERXToggle];        // Next Temp port for COMP1A
  OutBitNext1A = OutBitTable[TIMERXToggle];          // Next Temp bitmask for COMP1A
  OutPortNext1B = OutPortTable[TIMERXToggle+10];     // Next Temp port for COMP1B
  OutBitNext1B = OutBitTable[TIMERXToggle+10];       // Next Temp bitmask for COMP1B
}

void ServoSetup()
{
  // Timer 1 setup(16 bit):
  TCCR1A = 0;                     // Normal counting mode
  TCCR1B = 2;                     // Set prescaler to 1
  TCNT1 = 0;                      // Clear timer count
  TIFR1 = 255;                    // Clear  pending interrupts
  TIMSK1 = 6;                     // Enable the output compare A and B interrupt 
  // Timer 2 setup(8 bit):
  TCCRXA = 0;                     // Normal counting mode
  TCCRXB = 6;                     // Set prescaler to 256
  TCNTX = 0;                      // Clear timer count
  TIFRX = 255;                    // Clear pending interrupts
  TIMSKX = 6;                     // Enable the output compare A and B interrupt
  OCRXA = 106;                     // 93 Set counter A for about 500us before counter B below; 106 for 220us
  OCRXB = 137;                    // Set counter B for about 2000us (137 is 22ms, 124 20ms/10, where 20ms is 50Hz);

  timersRunning = true;
}

void initServoOutputs()
{
  for(iCount=2;iCount< NO_OF_OUTPUT_CHANNELS+2;iCount++) pinMode(iCount, OUTPUT);    // Set all pins used to output:
}

void defaultServoValues()
{
  for (int i=0; i < NO_OF_INPUT_CHANNELS; i++)
  {
    if (inputChannelMap[i] == THRO )
    {
      pwmPos[i] = DSMX_2K_LOW + DSMX_2K_OFFSET;
    }
    else
    {
      pwmPos[i] = DSMX_2K_CENTER + DSMX_2K_OFFSET;
    }
    ServoPW[i] = pwmPos[i];
  }
}

///////////////////////// SRXL2 channel interface //////////////////////////////

void userProvidedReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafe)
{
  int noOfChvalues = 0;

  if (isFailsafe)
  {
    defaultServoValues();
  }
  else
  {
    // Get channel values.
    for (int i=0; i < NO_OF_INPUT_CHANNELS; i++)
    {
      pwmPos[i] = srxlChData.values[inputChannelMap[i]] >> 5;    // 16-bit to 11-bit range (0 - 2048)
      if (pwmPos[i] == 0)
      {
        if (inputChannelMap[i] == THRO )
        {
          pwmPos[i] = DSMX_2K_LOW;
        }
        else
        {
          pwmPos[i] = DSMX_2K_CENTER;
        }
      }
      else
      {
        noOfChvalues++; // Count valid data
      }
      pwmPos[i] += DSMX_2K_OFFSET;

      // Did we get any valid data ??
      if (noOfChvalues > 0) validServoPacket = true;
    }
  }
}

  void uartSetBaud(uint8_t uart, uint32_t baudRate) // Automatic adjust SRXL2 baudrate.
  {
    // Not supported yet
  }

  void uartTransmit(uint8_t uart, uint8_t* pBuffer, uint8_t length)
  {
    for (uint8_t i=0; i < length; i++)
    {
      srxl2port.write(pBuffer[i]);
    }
  }
 
