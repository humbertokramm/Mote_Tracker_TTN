
// Algoritmo para UART por Software

#include "stdint.h"

#ifndef SW_UART_H
#define SW_UART_H

// Callback no final da entrada do frame. Fornece buffer de dados e número de bytes recebido 
extern void ReadyByteFrame(void);

void InterruptPinRX(void);

void InterruptTimerUART(void);
void clearInterruptTimerUART(void);
void reloadTimer(uint8_t setTimerValue);
void clearInterruptPinRX(void);
void printfOled(char value);

void InitSwUart(void);

uint8_t outFIFO(void);
uint8_t emptyFIFO(void);
uint8_t silentRX(void);

#endif