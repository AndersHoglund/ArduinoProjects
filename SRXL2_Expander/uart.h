#pragma once
 
void uartSetBaud(uint8_t uart, uint32_t baudRate);
void uartTransmit(uint8_t uart, uint8_t* pBuffer, uint8_t length);
