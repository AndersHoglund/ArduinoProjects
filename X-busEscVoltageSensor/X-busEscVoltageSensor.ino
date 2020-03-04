// Spektrum X-BUS Telemetry ESC voltage sensor

#include <Wire.h>

// select the analog voltage input pin
#define SENSOR_PIN A0    

// Define voltage divider resistor values
#define RESISTOR_HIGH (330) // kOhms
#define RESISTOR_LOW (33)

#define ADC_SCALE 5/1023
#define SCALE ((double)((RESISTOR_LOW + RESISTOR_HIGH) / RESISTOR_LOW) * ADC_SCALE)

#define INT8 char
#define INT16 short int
#define UINT8 unsigned char
#define UINT16 unsigned short int

#define I2C_SDA_PIN A4
#define I2C_SCL_PIN A5
#define I2C_SCL_STRETCH_PIN 12

//Select which TM frame to use
#define USE_ESC_FRAME
//#define USE_RPM_FRAME

#define TELE_DEVICE_ESC (0x20) // ESC
#define TELE_DEVICE_RPM (0x7e) // RPM

#define NO_DATA 0xff
#define UINT_NO_DATA_LSB 0x7fff
#define UINT_NO_DATA_MSB 0xff7f

typedef union
{
  unsigned char raw[2];
  unsigned int value;
} msb_u;

typedef struct
{
  UINT8 identifier;       // Source device = 0x20
  UINT8 sID;              // Secondary ID
  UINT16 RPM;             // RPM, 10RPM (0-655340 RPM).    0xFFFF --> "No data"
  UINT16 voltsInput;      // Volts, 0.01v (0-655.34V).     0xFFFF --> "No data"
  UINT16 tempFET;         // Temperature, 0.1C (0-999.8C)  0xFFFF --> "No data"
  UINT16 currentMotor;    // Current, 10mA (0-655.34A).    0xFFFF --> "No data"
  UINT16 tempBEC;         // Temperature, 0.1C (0-999.8C)  0x7FFF --> "No data"
  UINT8 currentBEC;       // BEC Current, 100mA (0-25.4A). 0xFF ----> "No data"
  UINT8 voltsBEC;         // BEC Volts, 0.05V (0-12.70V).  0xFF ----> "No data"
  UINT8 throttle;         // 0.5% (0-127%).                0xFF ----> "No data"
  UINT8 powerOut;         // Power Output, 0.5% (0-127%).  0xFF ----> "No data"
} STRU_TELE_ESC;

typedef struct
{
  UINT8 identifier;        // Source device = 0x7E
  UINT8 sID;               // Secondary ID
  UINT16 microseconds;     // microseconds between pulse leading edges
  UINT16 volts;            // 0.01V increments
  INT16 temperature;       // degrees F
  INT8 dBm_A;              // Average signal for A antenna in dBm
  INT8 dBm_B;              // Average signal for B antenna in dBm.
                           // If only 1 antenna, set B = A
} STRU_TELE_RPM;

typedef union
{
  UINT8 raw[16];
  //STRU_TELE_RTC rtc;
  //STRU_TELE_QOS qos;
  STRU_TELE_RPM rpm;
  //STRU_TELE_FRAMEDATA frame;
  //STRU_TELE_ALT alt;
  //STRU_TELE_SPEED speed;
  //STRU_TELE_ENERGY_DUAL eDual;
  //STRU_TELE_VARIO_S varioS;
  //STRU_TELE_G_METER accel;
  //STRU_TELE_JETCAT jetcat;
  //STRU_TELE_JETCAT2 jetcat2;
  //STRU_TELE_GPS_LOC gpsloc;
  //STRU_TELE_GPS_STAT gpsstat;
  //STRU_TELE_GYRO gyro;
  //STRU_TELE_ATTMAG attMag;
  //STRU_TELE_POWERBOX powerBox;
  STRU_TELE_ESC escGeneric;
  //STRU_TELE_LAPTIMER lapTimer;
  //STRU_TELE_TEXTGEN textgen;
  //STRU_TELE_FUEL fuel;
  //STRU_TELE_MAH mAh;
  //STRU_TELE_DIGITAL_AIR digAir;
  //STRU_TELE_STRAIN strain;
  //STRU_TELE_LIPOMON lipomon;
  //STRU_TELE_LIPOMON_14 lipomon14;
  //STRU_TELE_USER_16SU user_16SU;
  //STRU_TELE_USER_16SU32U user_16SU32U;
  //STRU_TELE_USER_16SU32S user_16SU32S;
  //STRU_TELE_USER_16U32SU user_16U32SU;
} UN_TELEMETRY;

#ifdef USE_ESC_FRAME
#define IDENTIFIER  TELE_DEVICE_ESC
#endif

#ifdef USE_RPM_FRAME
#define IDENTIFIER  TELE_DEVICE_RPM
#endif


// Globals

int sensorValue = 0;  // variable to store the value coming from the sensor
double scale = SCALE;
int once = 1;

UN_TELEMETRY TmBuffer = {IDENTIFIER, 0, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA, NO_DATA};


unsigned int LsbToMsb(unsigned int i)
{
  msb_u b;
  unsigned char temp;

  b.value = i;
  //Swap bytes
  temp     = b.raw[0];
  b.raw[0] = b.raw[1];
  b.raw[1] = temp;

  return(b.value);
}

void requestEvent()
{
  Wire.write(TmBuffer.raw, sizeof(TmBuffer) );
}

void setup()
{
  // All this work if Rx is powered up 5s after the Arduino Nano, fails if powered up at the same time. Clock stretching does not help.
  // Takes up to 4-5s to get Arduino Nano I2C up and running. Spektrum 8010T starts a single poll sequence after some 350mS, and thus gets no respons.   
  // Spektrum says: "We don't recommend clock stretching with the T-series receivers."
  // Looks like SCL is driven high active, not by a pullup, on the scope. I.e. not I2C compliant.... Or?
  
//  pinMode(I2C_SCL_STRETCH_PIN,OUTPUT);
//  digitalWrite(I2C_SCL_STRETCH_PIN,LOW);           // Force SCL low to keep Master idling till we are up and running.
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);           // Debug LED and scope trigger
  
  Wire.begin(TELE_DEVICE_ESC);    // join i2c bus with a slave address
  Wire.onRequest(requestEvent);   // register event
}


void loop()
{
  if (once)
  {
    once = 0;
    digitalWrite(13, HIGH);
//    // Release SCL
//    pinMode(I2C_SCL_STRETCH_PIN,INPUT);     // Open drain
//    digitalWrite(I2C_SCL_STRETCH_PIN,HIGH);
  }

  // read the value from the ADC:  
  sensorValue = analogRead(SENSOR_PIN);
  
  // compute real voltage
  double voltage = sensorValue * scale;
  unsigned short int centiVoltage= voltage *100;

#ifdef USE_ESC_FRAME
  TmBuffer.escGeneric.voltsInput = LsbToMsb(centiVoltage);
  TmBuffer.escGeneric.tempBEC    = UINT_NO_DATA_MSB; // Odd...
#endif

#ifdef USE_RPM_FRAME
  TmBuffer.rpm.volts = LsbToMsb(centiVoltage);
#endif

}
