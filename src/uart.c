
#include "uart.h"
#include "main.h"
bool rx_buf_ready = false;
bool rxThreshold = false;
char rx_buf[MAX_STRBUF_SIZE];

// Initialize EUSCI
void InitEusci(void)
{
  
    // Configure UART 9600 @ SMCLK 1MHz
    // http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;// EUSCI_A_UART_CLOCKSOURCE_SMCLK
    param.clockPrescalar = 6; //16MHz 8
    param.firstModReg = 8; //16MHz 10
    param.secondModReg = 17; // 16MHz 247
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;// 16MHz EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE,
                                EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable USCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
                                 EUSCI_A_UART_RECEIVE_INTERRUPT);      // Enable interrupt
}


// Transmits string buffer through EUSCI UART
void TransmitString(char *str)
{
    int i = 0;
    for(i = 0; i < strlen(str); i++)
    {
        if (str[i] != 0)
        {
            // Transmit Character
            while (EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY));
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, str[i]);
        }
    }
}


// Receives strings terminated with \n
void ReceiveString(char data) {
    static bool rxInProgress = false;
    static unsigned int charCnt = 0;

    if(!rxInProgress){
        if ((data != '\n') ){
            rxInProgress = true;
            charCnt = 0;
            rx_buf[charCnt] = data;
        }

    }else{ // in progress
        charCnt++;
        if((data != '\n')){
            if (charCnt >= MAX_STRBUF_SIZE){
                rxInProgress = false;
            }else{
                rx_buf[charCnt] = data;
            }
        }else{
            rxInProgress = false;
            rx_buf[charCnt] = '\0';
            // String receive complete
            rx_buf_ready = true;
        }
    }
}


// EUSCI interrupt service routine

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)

{
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            // Read buffer
            ReceiveString(UCA0RXBUF);
            TransmitString("DATA");
            if (rxThreshold)
            {
                if (rx_buf_ready)
                {
                    //threshold = _Q8(strtof(rx_buf, NULL));
                    rxThreshold = false;
                    rx_buf_ready = false;
                }
            }
            if (rx_buf_ready && !rxThreshold)
            {
                if (strcmp(rx_buf,"THRESH")==0)
                {
                    rxThreshold = true;
                }
                rx_buf_ready = false;
            }

            break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}

