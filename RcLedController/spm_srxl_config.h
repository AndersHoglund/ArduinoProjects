/*
MIT License
Copyright (c) 2019 Horizon Hobby, LLC
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// This file is automatically included within spm_srxl.h -- do not include elsewhere!

#ifndef _SRXL_CONFIG_H_
#define _SRXL_CONFIG_H_

#define SRXL_CRC_OPTIMIZE_MODE SRXL_CRC_OPTIMIZE_SPEED

//### USER PROVIDED HEADER FUNCTIONS AND FORWARD DECLARATIONS ###

#ifdef __cplusplus
extern "C" {
#endif

// User included headers/declarations to access interface functions required below
void userProvidedFillSrxlTelemetry(SrxlTelemetryData* pTelemetry);
void userProvidedReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafe);
void userProvidedHandleVtxData(SrxlVtxData* pVtxData);
void uartSetBaud(uint8_t uart, uint32_t baudRate);
void uartTransmit(uint8_t uart, uint8_t* pBuffer, uint8_t length);

//### USER CONFIGURATION ###

// Set this value to the number of physically separate SRXL buses on the device
#define SRXL_NUM_OF_BUSES           1

// Set this to the appropriate device ID (See Section 7.1.1 in Spektrum Bi-Directional SRXL Documentation).
// Typical values are:
//    Flight Controller = 0x31 (or possibly 0x30 if connected to Base Receiver instead of Remote Receiver)
//    Smart ESC = 0x40
//    VTX = 0x81

// Available Device Types
#define SRXL_DEVICE_NONE       (SRXL_DEFAULT_ID_OF_TYPE[SrxlDevType_None])
#define SRXL_DEVICE_BASE_RX    (SRXL_DEFAULT_ID_OF_TYPE[SrxlDevType_Receiver])
#define SRXL_DEVICE_REMOTE_RX  (SRXL_DEFAULT_ID_OF_TYPE[SrxlDevType_RemoteReceiver])
#define SRXL_DEVICE_FC         (SRXL_DEFAULT_ID_OF_TYPE[SrxlDevType_FlightController])
#define SRXL_DEVICE_ESC        (SRXL_DEFAULT_ID_OF_TYPE[SrxlDevType_ESC])
#define SRXL_DEVICE_SERVO1     (SRXL_DEFAULT_ID_OF_TYPE[SrxlDevType_SRXLServo1])
#define SRXL_DEVICE_SERVO2     (SRXL_DEFAULT_ID_OF_TYPE[SrxlDevType_SRXLServo2])
#define SRXL_DEVICE_VTX        (SRXL_DEFAULT_ID_OF_TYPE[SrxlDevType_VTX])
#define SRXL_DEVICE_BRDC       (SRXL_DEFAULT_ID_OF_TYPE[SrxlDevType_Broadcast])

// Select actual Device id to use from above list.
#define SRXL_DEVICE_ID         SRXL_DEVICE_SERVO1
#define SRXL_REQ_DEVICE_ID     SRXL_DEVICE_FC

// Set this to the desired priority level for sending telemetry, ranging from 0 to 100.
// Generally, this number should be 10 times the number of different telemetry packets to regularly send.
// If there are telemetry messages that should be sent more often, increase this value.
// If there are messages that are rarely sent, add less than 10 for those.
// For example, if you had two normal priority messages and one that you plan to send twice as often as those,
// you could set the priority to 40 (10 + 10 + 2*10)
// NOTE: This value is not used internally -- it is passed as a parameter to srxlInit() in the example app
#define SRXL_DEVICE_PRIORITY        0 // No SRXL2 telemtry over this device

// Set these information bits based on the capabilities of the device.
// The only bit currently applicable to third-party devices is the SRXL_DEVINFO_FWD_PROG_SUPPORT flag,
// which should be set if you would like to allow Forward Programming of the device via SRXL pass-through.
#define SRXL_DEVICE_INFO            (SRXL_DEVINFO_NO_RF)//SRXL_DEVINFO_NO_RF

// Set this value to 0 for 115200 baud only, or 1 for 400000 baud support
#define SRXL_SUPPORTED_BAUD_RATES   0

// Set this value to choose which code to include for CRC computation. Choices are:
//    SRXL_CRC_OPTIMIZE_SPEED   -- Uses table lookup for CRC computation (requires 512 const bytes for CRC table)
//    SRXL_CRC_OPTIMIZE_SIZE    -- Uses bitwise operations for smaller code size but slower execution
//    SRXL_CRC_OPTIMIZE_STM_HW  -- Uses STM32 register-level hardware acceleration (only available on STM32F30x devices for now)
//    SRXL_CRC_OPTIMIZE_STM_HAL -- Uses STM32Cube HAL driver for hardware acceleration (only available on STM32F3/F7) -- see srxlCrc16() for details on HAL config
#define SRXL_CRC_OPTIMIZE_MODE      SRXL_CRC_OPTIMIZE_SPEED

// If using STM32 hardware CRC acceleration above, set this flag to the target family. Choices are:
//    SRXL_STM_TARGET_F3
//    SRXL_STM_TARGET_F7
//#define SRXL_STM_TARGET_FAMILY      SRXL_STM_TARGET_F3

// If using SRXL_CRC_OPTIMIZE_STM_HW and the CRC hardware is shared with non-SRXL code, then
// uncomment the following flag to save and restore the CRC registers after use by SRXL:
//#define SRXL_SAVE_HW_CRC_CONTEXT

// Uncomment the following flag if your code must support Forward Programming received via SRXL
//#define SRXL_INCLUDE_FWD_PGM_CODE

//### USER PROVIDED INTERFACE FUNCTIONS ###

// User-provided routine to change the baud rate settings on the given UART:
// uart - the same uint8_t value as the uart parameter passed to srxlInit()
// baudRate - the actual baud rate (currently either 115200 or 400000)
static inline void srxlChangeBaudRate(uint8_t uart, uint32_t baudRate)
{
    uartSetBaud(uart, baudRate);
}

// User-provided routine to actually transmit a packet on the given UART:
// uart - the same uint8_t value as the uart parameter passed to srxlInit()
// pBuffer - a pointer to an array of uint8_t values to send over the UART
// length - the number of bytes contained in pBuffer that should be sent
static inline void srxlSendOnUart(uint8_t uart, uint8_t* pBuffer, uint8_t length)
{
    uartTransmit(uart, pBuffer, length);
}

// User-provided callback routine to fill in the telemetry data to send to the master when requested:
// pTelemetryData - a pointer to the 16-byte SrxlTelemetryData transmit buffer to populate
// NOTE: srxlTelemData is available as a global variable, so the memcpy line commented out below
// could be used if you would prefer to just populate that with the next outgoing telemetry packet.
static inline void srxlFillTelemetry(SrxlTelemetryData* pTelemetryData)
{
    //memcpy(pTelemetryData, srxlTelemData, sizeof(SrxlTelemetryData));
    //userProvidedFillSrxlTelemetry(pTelemetryData);
}

// User-provided callback routine that is called whenever a control data packet is received:
// pChannelData - a pointer to the received SrxlChannelData structure for manual parsing
// isFailsafe - true if channel data is set to failsafe values, else false.
// NOTE: srxlChData is available as a global variable that contains all of the latest values
// for channel data, so this callback is intended to be used if more control is desired.
// It might make sense to only use this to trigger your own handling of the received servo values.
static inline void srxlReceivedChannelData(SrxlChannelData* pChannelData, bool isFailsafe)
{
    userProvidedReceivedChannelData(pChannelData, isFailsafe);
}

// User-provided callback routine to handle reception of a bound data report (either requested or unprompted).
// Return true if you want this bind information set automatically for all other receivers on all SRXL buses.
static inline bool srxlOnBind(SrxlFullID device, SrxlBindData info)
{
    // TODO: Add custom handling of bound data report here if needed
    return true;
}

// User-provided callback routine to handle reception of a VTX control packet.
static inline void srxlOnVtx(SrxlVtxData* pVtxData)
{
    //userProvidedHandleVtxData(pVtxData);
}

// Optional user-provided callback routine to handle Forward Programming command locally if supported
#ifdef SRXL_INCLUDE_FWD_PGM_CODE
static inline void srxlOnFwdPgm(uint8_t* pData, uint8_t dataLength)
{
    // TODO: Pass data to Forward Programming library
}
#endif // SRXL_INCLUDE_FWD_PGM_CODE

// User-provided routine to enter a critical section (only needed with multiple buses or if HW CRC is used externally)
static inline void srxlEnterCriticalSection(void)
{
}

// User-provided routine to exit a critical section (only needed with multiple buses or if HW CRC is used externally)
static inline void srxlExitCriticalSection(void)
{
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _SRXL_CONFIG_H_
