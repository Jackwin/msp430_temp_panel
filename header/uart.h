#include <msp430fr4133.h>
#include <driverlib.h>
#ifndef _UART_H_
#define _UART_H_



void InitEusci(void);
void ReceiveString(char);
void TransmitString(char *str);

#endif
