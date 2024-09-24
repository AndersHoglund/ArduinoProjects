#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define USE_SOFTWARE_SERIAL

// Only support DSMx SERIALRX_SPEKTRUM2048:
#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEK_FRAME_SIZE                      16
#define SPEK_CHAN_SHIFT                       3
#define SPEK_CHAN_MASK                     0x07
#define SPEKTRUM_NEEDED_FRAME_INTERVAL        5
#define SPEKTRUM_BAUDRATE                115200

void setupSerialRx();
void getSerialRxPwm(unsigned long currentTime, uint8_t rcChannel, uint16_t * pwmValuePtr);

#ifdef __cplusplus
} // extern "C"
#endif
