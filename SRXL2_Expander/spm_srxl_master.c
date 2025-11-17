#include <string.h>
#include <stdint.h>

#include "spm_srxl.h"

#ifdef SRXL_INCLUDE_MASTER_CODE

void srxlRunMaster(SrxlBus* pBus) {};

void srxlInitReceiver(uint8_t deviceID, uint8_t info) {};
void srxlSetOutgoingChannelMask(uint32_t mask) {};
void srxlTryToBind(SrxlBindData srxlBindInfo) {};

void srxlTelemetrySent() {};
void srxlSetTelemetryTxEnable(bool en) {};
void srxlSuppressInternalTelemetry(SrxlTelemetryData* payload) {};

#endif