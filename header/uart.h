#ifndef _UART_H_
#define _UART_H_

#include <msp430fr4133.h>
#include <driverlib.h>
#include "main.h"


void InitEusci(void);
void ReceiveString(char);
void TransmitString(char *str);

#endif
