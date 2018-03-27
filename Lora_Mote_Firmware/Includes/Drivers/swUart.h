
// Algoritmo para UART por Software

#include "stdint.h"

#ifndef SW_UART_H
#define SW_UART_H

// Callback no final da entrada do frame. Fornece buffer de dados e número de bytes recebido 
extern void ReadyByteFrame(int8_t* buffer_rx, uint8_t numBytes);

void InterruptPinRX(void);

void InterruptTimerUART(void);

#endif