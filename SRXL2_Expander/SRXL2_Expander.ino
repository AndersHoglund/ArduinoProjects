//////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SRXL2 decoder example for Base Receivers and ATMega328P. Used receiver in this exampel is the AR6610T.
// Channels are locked to these pins:
// Ch0=Pin2, Ch1=Pin3, Ch2=Pin4, Ch3=Pin5, Ch4=Pin6, Ch5=Pin7, Ch6=Pin8, Ch7=Pin9, Ch8=Pin10, Ch9=Pin11
// Ch10=Pin12, Ch11=Pin13, (Ch12=PinA0, Ch13=PinA1, Ch14=PinA2, Ch15=PinA3, Ch16=PinA4, Ch17=PinA5)
//
// 
//////////////////////////////////////////////////////////////////////////////////////////////////////

 #include <avr/interrupt.h>
   #include "spm_srxl.h"
   #include <avr/wdt.h>//watchdog
   
  
 
 #define SRXL2_PORT_BAUDRATE_DEFAULT 115200
 #define SRXL2_FRAME_TIMEOUT 22 
 #define srxl2port Serial
 unsigned long currentTime;
 unsigned long prevPwmTime = 0;
 const long pwmInterval = 22;

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

 

  uint16_t pwmPos = 8991;    // variable to store the PWM servo position  
  uint16_t pwmPos1 = 8991;    // 
  uint16_t pwmPos2 = 8991;    //
  uint16_t pwmPos3 = 8991;    // 
  uint16_t pwmPos4 = 8991;    // 
  uint16_t pwmPos5 = 8991;    // 
  uint16_t pwmPos6 = 8991;    // 
  uint16_t pwmPos7 = 8991;    // 
  uint16_t pwmPos8 = 8991;    // 
  uint16_t pwmPos9 = 8991;    // 
  uint16_t pwmPos10 = 8991;    //
  uint16_t pwmPos11 = 8991;    // 
  
 
#define YES 1
#define NO 0
#define UseJitterCompensation YES

static byte Jitter;
static byte Jitter2;
static byte Jitter3;
static byte Jitter4;
//static byte RealTime5s;
static unsigned int iCount;
static volatile uint8_t *OutPortTable[20] = {&PORTD,&PORTD,&PORTD,&PORTD,&PORTD,&PORTD,&PORTB,&PORTB,&PORTB,&PORTB,&PORTB,&PORTB,&PORTC,&PORTC,&PORTC,&PORTC,&PORTC,&PORTC,&PORTD,&PORTD};
static uint8_t OutBitTable[20] = {4,8,16,32,64,128,1,2,4,8,16,32,1,2,4,8,16,32,1,2};
static unsigned int ServoPW[20] = {8991,8991,8991,8991,8991,8991,8991,8991,8991,8991,8991,8991,8991,8991,8991,8991,8991,8991,8991,8991};//10015
static byte Timer2Toggle;
static volatile uint8_t *OutPort1A = &PORTD;
static volatile uint8_t *OutPort1B = &PORTB;
static uint8_t OutBit1A = 4;
static uint8_t OutBit1B = 16;
static volatile uint8_t *OutPortNext1A = &PORTD;
static volatile uint8_t *OutPortNext1B = &PORTB;
static uint8_t OutBitNext1A = 4;
static uint8_t OutBitNext1B = 16;

static long ServoStepsHD[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static long StepsToGo[20] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
static int ChannelCount;


void setup()
{
srxl2port.begin(SRXL2_PORT_BAUDRATE_DEFAULT);
      
 srxlInitDevice(SRXL_DEVICE_ID, SRXL_DEVICE_PRIORITY, SRXL_DEVICE_INFO, 0x01000001);//
 srxlInitBus(0, 1, SRXL_SUPPORTED_BAUD_RATES);// 
  
  ServoSetup();     //Initiate timers and misc.
 wdt_enable(WDTO_250MS);   // Watchdog auf 0,25 s stellen und starten. Set watchdog to 0.25 s and start.
 
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
  ServoPW[0] = (pwmPos); 
  ServoPW[1] = (pwmPos1);
  ServoPW[2] = (pwmPos2);
  ServoPW[3] = (pwmPos3);
  ServoPW[4] = (pwmPos4);
  ServoPW[5] = (pwmPos5);
  ServoPW[6] = (pwmPos6);
  ServoPW[7] = (pwmPos7);
  ServoPW[8] = (pwmPos8);
  ServoPW[9] = (pwmPos9);
  ServoPW[10] =(pwmPos10);
  ServoPW[11] =(pwmPos11);
  PPM();                                  // Do servo move
  }
 
}



volatile void PPM() //Move servos every 22ms to the desired position.
{
  wdt_reset();// Watchdog zurücksetzen. Reset the Watchdog.
    
  for(ChannelCount = 0; ChannelCount < 18; ChannelCount++)
  {
    if(StepsToGo[ChannelCount] > 0)
    {
      ServoPW[ChannelCount] += ServoStepsHD[ChannelCount];
      StepsToGo[ChannelCount] --;
    }
    
  }
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

ISR(TIMER2_COMPA_vect) // Interrupt routine for timer 2 compare A. Used for timing 50Hz for each servo.
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

ISR(TIMER2_COMPB_vect) // Interrupt routine for timer 2 compare B. Used for timing 50Hz for each servo.
{ 
  TIFR1 = 255;                                       // Clear  pending interrupts
  #if UseJitterCompensation == YES
    Jitter3 = TCNT1L-100;
    if(Jitter3 == 137){asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");}
    if(Jitter3 == 136){asm volatile("nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t");}
    if(Jitter3 == 135)  {asm volatile("nop\n\tnop\n\tnop\n\t");}
    if(Jitter3 == 134){asm volatile("nop\n\t");}
    if(Jitter3 == 133){asm volatile("nop\n\t");}
  #endif
  TCNT1 = 0;                                         // Restart counter for timer1
  TCNT2 = 0;                                         // Restart counter for timer2
  sei();
  *OutPort1A &= ~OutBit1A;                           // Set pulse low to if not done already
  *OutPort1B &= ~OutBit1B;                           // Set pulse low to if not done already
  OutPort1A = OutPortTable[Timer2Toggle];            // Temp port for COMP1A
  OutBit1A = OutBitTable[Timer2Toggle];              // Temp bitmask for COMP1A
  OutPort1B = OutPortTable[Timer2Toggle+10];         // Temp port for COMP1B
  OutBit1B = OutBitTable[Timer2Toggle+10];           // Temp bitmask for COMP1B
  
  OCR1A = ServoPW[Timer2Toggle]-8020;
  OCR1B = ServoPW[Timer2Toggle+10]-8015; 
  Timer2Toggle++;                                    // Next servo in line.
  if(Timer2Toggle==10)
  { 
    Timer2Toggle = 0;                                // If next servo is grater than 9, start on 0 again.
                                   
  }
  OutPortNext1A = OutPortTable[Timer2Toggle];        // Next Temp port for COMP1A
  OutBitNext1A = OutBitTable[Timer2Toggle];          // Next Temp bitmask for COMP1A
  OutPortNext1B = OutPortTable[Timer2Toggle+10];     // Next Temp port for COMP1B
  OutBitNext1B = OutBitTable[Timer2Toggle+10];       // Next Temp bitmask for COMP1B
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
  TCCR2A = 0;                     // Normal counting mode 
  TCCR2B = 6;                     // Set prescaler to 256
  TCNT2 = 0;                      // Clear timer count 
  TIFR2 = 255;                    // Clear pending interrupts
  TIMSK2 = 6;                     // Enable the output compare A and B interrupt 
  OCR2A = 106;                     // 93 Set counter A for about 500us before counter B below; 106 for 220us
  OCR2B = 137;                    // Set counter B for about 2000us (137 is 22ms, 124 20ms/10, where 20ms is 50Hz);
  
  
    for(iCount=2;iCount<14;iCount++) pinMode(iCount, OUTPUT);    // Set all pins used to output:
    OutPortTable[18] = &PORTC;    // In 18 channel mode set channel 18 and 19 to a dummy pin that does not exist.
    OutPortTable[19] = &PORTC;
    OutBitTable[18] = 128;
    OutBitTable[19] = 128;
    
  DDRC = 63;                      //Set analog pins A0 - A5 as digital output.
}

///////////////////////// SRXL2 channel interface //////////////////////////////


 void userProvidedReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafe)
 {
 // Get throttle channel value and convert to 1000 - 1500 - 2000 pwm range
 
  pwmPos = srxlChData.values[THRO] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
 if (pwmPos == 0){pwmPos = 1024; } // Uncomment when use as aileron. Comment out this when used as throttle cannel.
  pwmPos += 8991;
  
   // Get Aile channel value and convert to 1000 - 1500 - 2000 pwm range
   pwmPos1 = srxlChData.values[AILE] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
    if (pwmPos1 == 0){pwmPos1 = 1024; }
    pwmPos1 += 8991;

   // Get elevator channel value and convert to 1000 - 1500 - 2000 pwm range
  pwmPos2 = srxlChData.values[ELEV] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
  if (pwmPos2 == 0){pwmPos2 = 1024; }
  pwmPos2 += 8991;

// Get yaw channel value and convert to 1000 - 1500 - 2000 pwm range
  pwmPos3 = srxlChData.values[YAW] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
  if (pwmPos3 == 0){pwmPos3 = 1024; }
  pwmPos3 += 8991;

   // Get gear channel value and convert to 1000 - 1500 - 2000 pwm range
  pwmPos4 = srxlChData.values[GEAR] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
  if (pwmPos4 == 0){pwmPos4 = 1024; }
  pwmPos4 += 8991;

  // Get Aux1 channel value and convert to 1000 - 1500 - 2000 pwm range
   pwmPos5 = srxlChData.values[AUX1] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
   if (pwmPos5 == 0){pwmPos5 = 1024; }
  pwmPos5 += 8991; 

  // Get Aux2 channel value and convert to 1000 - 1500 - 2000 pwm range
   pwmPos6 = srxlChData.values[AUX2] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
  if (isFailsafe == true){pwmPos6 = 0; } //Failsafe throttle out
  pwmPos6 += 8991;

 // Get Aux2 channel value and convert to 1000 - 1500 - 2000 pwm range
   pwmPos7 = srxlChData.values[AUX3] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
  if (isFailsafe == true){pwmPos7 = 1024; }
  pwmPos7 += 8991;

  // Get Aux2 channel value and convert to 1000 - 1500 - 2000 pwm range
   pwmPos8 = srxlChData.values[AUX4] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
  if (isFailsafe == true){pwmPos8 = 1024; }
  pwmPos8 += 8991;

  // Get Aux2 channel value and convert to 1000 - 1500 - 2000 pwm range
   pwmPos9 = srxlChData.values[AUX5] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
   if (isFailsafe == true){pwmPos9 = 1024; }
   pwmPos9 += 8991;

  // Get Aux2 channel value and convert to 1000 - 1500 - 2000 pwm range
   pwmPos10 = srxlChData.values[AUX6] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
    if (isFailsafe == true){pwmPos10 = 1024; }
    pwmPos10 += 8991;

  // Get Aux2 channel value and convert to 1000 - 1500 - 2000 pwm range
   pwmPos11 = srxlChData.values[AUX7] >> 5;    // 16-bit to 11-bit range (0 - 2048) 
    if (isFailsafe == true){pwmPos11 = 1024; }
    pwmPos11 += 8991;

   
  
 
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
  srxl2port.flush();
 }
 
