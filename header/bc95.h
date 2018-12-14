
#ifndef BC95_H_
#define BC95_H_

#include <msp430fr4133.h>
#include <driverlib.h>
#include "main.h"
#include "uart.h"

extern char rx_buf[MAX_STRBUF_SIZE];
void BC95Init(void);
void BC95ConnectCloud(void);
void ClearRxBuffer(void);

typedef struct
{
    uint8_t CSQ;
    uint8_t Socketnum;
    uint8_t reclen;
    uint8_t res;
    uint8_t recdatalen[10];
    uint8_t recdata[100];
    uint8_t uart1len[10];
    uint8_t senddata[100];
} BC95;


#endif
