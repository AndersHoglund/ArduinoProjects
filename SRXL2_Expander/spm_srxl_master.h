#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

void srxlRunMaster(SrxlBus* pBus);

void srxlInitReceiver(uint8_t deviceID, uint8_t info);
void srxlSetOutgoingChannelMask(uint32_t mask);
void srxlTryToBind(SrxlBindData srxlBindInfo);

void srxlTelemetrySent();
void srxlSetTelemetryTxEnable(bool en);
void srxlSuppressInternalTelemetry(SrxlTelemetryData* payload);

#ifdef __cplusplus
} // extern "C"
#endif