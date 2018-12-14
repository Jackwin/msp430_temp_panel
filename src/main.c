#include <msp430.h>
#include <driverlib.h>
#include <spi_intf.h>
#include <string.h>
#include <stdio.h>
//#include "si446x.h"
#include "bsp.h"
#include "tx.h"
#include "StopWatchMode.h"
#include "TempSensorMode.h"
#include "hal_LCD_4463.h"
#include "main.h"
//#include "uart.h"
//#include "bc95.h"


// UART Defines
#define UART_TXD_PORT        GPIO_PORT_P1
#define UART_TXD_PIN         GPIO_PIN0
#define UART_RXD_PORT        GPIO_PORT_P1
#define UART_RXD_PIN         GPIO_PIN1
#define UART_SELECT_FUNCTION GPIO_PRIMARY_MODULE_FUNCTION

// Define word access definitions to LCD memories
#define LCDMEMW ((int*)LCDMEM)

// Backup Memory variables to track states through LPM3.5
volatile unsigned char * mode = &BAKMEM4_L;                   // mode flag
volatile unsigned int holdCount = 0;



//32 pin LCD with newboard, numberic table for 7,8,9,10 LED
const char digit4463_7_10_test[10] =
{
  0XF5,  /* "0" LCD segments a+b+c+d+e+f+k+q */
  0X05,  /* "1" */
  0XB6,  /* "2" */
  0X97,  /* "3" */
  0X47,  /* "4" */
  0XD3,  /* "5" */
  0XF3,  /* "6" */
  0X85,  /* "7" */
  0XF7,  /* "8" */
  0XD7   /* "9" */
};

//32 pin LCD with newboard, numberic table for 1,2,3,4,5,6 LED
const char digit4463_1_6_test[10] =
{
  0XFA,  /* "0" LCD segments a+b+c+d+e+f+k+q */
  0X0A,  /* "1" */
  0XD6,  /* "2" */
  0X9E,  /* "3" */
  0X2E,  /* "4" */
  0XBC,  /* "5" */
  0XFC,  /* "6" */
  0X1A,  /* "7" */
  0XFE,  /* "8" */
  0XBE   /* "9" */
};

/**
 * main.c
 */

uint8_t buffer[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};




#define MAX_STRBUF_SIZE 128
#define NOP() _nop()
char rx_buf[MAX_STRBUF_SIZE];
void BC95ConnectCloud(void);
void BC95Init(void);
void InitEusci(void);
void TransmitString(char *str);
void ClearRxBuffer(void);
void DelayClock(uint16_t i);
//char mode;

//char rx_buf[MAX_STRBUF_SIZE];

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



int main(void)
{
  WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer
  WDT_A_hold(__MSP430_BASEADDRESS_WDT_A__);     // Stop WD

//    P5DIR |= 0x03;


  // buttonint();
  char *strx;

  //PM5CTL0 &= ~LOCKLPM5;



//  GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN4);
//  GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN5);
//  GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
//  GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN7);


  /*
      LCDMEM[0] = 0xff;
      LCDMEM[1] = 0xff;  //LED2
      LCDMEM[2] = 0xff;  //LED3
      LCDMEM[3] = 0xff;  //LED4
      LCDMEM[4] = 0xff;  //LED5
      LCDMEM[5] = 0xff;//0x0;
      LCDMEM[6] = 0xff;  //LED6
      LCDMEM[7] = 0xff;  //LED1
      LCDMEM[8] = 0xff;//0x0;
      LCDMEM[9] = 0xff;//0x0;
      LCDMEM[10] = 0xff; //LED10
      LCDMEM[11] = 0xff;//LED9
      LCDMEM[12] = 0xff;//LED8
      LCDMEM[13] = 0xff;//LED7
      */

  /* LCDMEM[7] = digit4463_1_6_test[4];
   LCDMEM[6] = digit4463_1_6_test[5];
   LCDMEM[1] = digit4463_1_6_test[6];
   LCDMEM[2] = digit4463_1_6_test[7];
  LCDMEM[3] = digit4463_1_6_test[8];
  LCDMEM[4] = digit4463_1_6_test[9];
  // LCDMEM[0] = 0xf8;
  LCDMEM[13] = digit4463_7_10_test[4]|0x08;
  LCDMEM[12] = digit4463_7_10_test[5]|0x08;
   LCDMEM[11] = digit4463_7_10_test[6]|0x08;
   LCDMEM[10] = digit4463_7_10_test[7]|0x08;*/

  // clearLCD();              // Clear all LCD segments
//   tempSensorModeInit();    // initialize temperature mode //have a bug

  //  tempSensoronchip(); //Debuged by zhaojz


  // clearLCD4463();              // Clear all LCD segments

  /*     LCDMEM[0] = 0xff;
       LCDMEM[1] = 0xff;
       LCDMEM[2] = 0xff;
       LCDMEM[3] = 0xff;
       LCDMEM[4] = 0xff;//0x00;
       LCDMEM[5] = 0xff;//0x0;
       LCDMEM[6] = 0xff;//0x0;
       LCDMEM[7] = 0xff;//0x0;
       LCDMEM[8] = 0xff;//0x0;
       LCDMEM[9] = 0xff;//0x0;
       LCDMEM[10] = 0xff;//0x0;
       LCDMEM[11] = 0xff;//0x0;
       LCDMEM[12] = 0xff;//0x0;
       LCDMEM[13] = 0xff;//0x0; */

  //resetSetTemperature();

 // Init_spi_B();
  InitGPIO();

    InitClock();
    InitLCD();
  Init_RTC();
  InitEusci();


//   System_Initial();

  __enable_interrupt();


  //stopWatchModeInit();     // Initialize stopwatch mode

  BC95Init();
  BC95ConnectCloud();
  while (1)
  {


     while(strx==NULL)
           {
             int i = 0xffff;
             int j = 0xffff;

            // TransmitString("AT\r\n");
             for (;i > 0; i--)
                 for (; j >0; j--)
                 ;
            //  strx=strstr((const char*)rx_buf,(const char*)"OK");
             // ClearRxBuffer();
           }



  }



//    Init_spi();

//    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
//    __delay_cycles(5);
//    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, TXData);
//    __delay_cycles(5);
//    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
  //SI446X_RESET();
//    System_Initial();
//    SI446X_RESET();
//    //SI446X_CONFIG_INIT();
//    SI446X_SET_POWER(0x7F);
//    //SI446X_PART_INFO(buffer);
//    SI446X_GET_PROPERTY_X(GLOBAL_XO_TUNE, 10, buffer);
//    SI446X_GET_PROPERTY_X(INT_CTL_ENABLE, 4, buffer);
//    SI446X_GET_PROPERTY_X(FRR_CTL_A_MODE, 4, buffer);
//    SI446X_GET_PROPERTY_X(SYNC_CONFIG, 5, buffer);
//    SI446X_GET_PROPERTY_X(PKT_CRC_CONFIG, 13, buffer);
//    SI446X_GET_PROPERTY_X(MODEM_MOD_TYPE, 13, buffer);
//    buffer[7] = SI446X_GET_PROPERTY_1(PKT_FIELD_1_LENGTH_7_0);
//    SI446X_PART_INFO(buffer);
  /*
    while(1)
    {
  //#if 0    //RX
  #if 1  //TX
        i++;
          if (i == 10)
          {
             RF_Tx();
              i = 0;
          }
  #else
          RF_Rx();
  #endif

        //P5OUT ^= 0x01;
  //      __delay_cycles(1000000);
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);

        //SI_CSN_LOW();
        //SPI_ExchangeByte(TXData);
        //SI_CSN_HIGH();
    }
  */
//  return 0;
}



/*
 * Clock System Initialization
 */
void InitClock()
{
  // Intializes the XT1 crystal oscillator
  //CS_turnOnXT1(CS_XT1_DRIVE_1);
  //CS_bypassXT1();

  // Set REFO as the DCO FLLL reference clock
  // REFO is internal 32.768 KHz
  CS_initClockSignal(
    CS_FLLREF,
    CS_REFOCLK_SELECT,
    CS_CLOCK_DIVIDER_1
  );

  //Set ACLK = REFO
  CS_initClockSignal(
    CS_ACLK,
    CS_REFOCLK_SELECT,
    CS_CLOCK_DIVIDER_1
  );

  // In default mode, SMCLK = 1MHz



}

/*
 * Real Time Clock counter Initialization
 */
void Init_RTC()
{
  // Set RTC modulo to 327-1 to trigger interrupt every ~10 ms
  RTC_setModulo(RTC_BASE, 32700);
  RTC_enableInterrupt(RTC_BASE, RTC_OVERFLOW_INTERRUPT);
  RTC_start(RTC_BASE, 0x2000);
}




void DelayClock(uint16_t i)
{
  uint16_t counter;

  for (counter = 0; counter < i; counter++)
      NOP();
}


/*
 * GPIO Initialization
 */
void InitGPIO()
{
  // Set all GPIO pins to output low to prevent floating input and reduce power consumption
  GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
  /*   GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
     GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
     GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
     GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
     GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
     GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
     GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);*/
  GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);

      GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
  /*    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
      GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
      GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
      GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
      GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
      GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
      GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);*/
  GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);

//    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN1);

  // Configure button S1 (P1.2) interrupt
//    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
//    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
//    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
//    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

  //Button fuction define:
  //   Temperature   |       Time
  //-----------------|--------------------
  //   S3      S4    |   S5       S6
  //   +       -     | Hour+    Minutes+
  //-----------------|---------------------
  // P2.4     P2.5   |  P2.6      P2.7
  //-----------------|---------------------

  // Configure button S3 (P2.4) interrupt
  GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN4, GPIO_HIGH_TO_LOW_TRANSITION);
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN4);
  GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN4);
  GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN4);

  // Configure button S4 (P2.5) interrupt
  GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN5);
  GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN5);
  GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN5);

  // Configure button S5 (P2.6) interrupt
  GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
  GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);

  // Configure button S6 (P2.7) interrupt
  GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN7, GPIO_HIGH_TO_LOW_TRANSITION);
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN7);
  GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN7);
  GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN7);


  // Disable BC95 reset
  GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0);
  GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);

  GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
  GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);



  // Set P4.1 and P4.2 as Secondary Module Function Input, LFXT.
//    GPIO_setAsPeripheralModuleFunctionInputPin(
//           GPIO_PORT_P4,
//           GPIO_PIN1 + GPIO_PIN2,
//           GPIO_PRIMARY_MODULE_FUNCTION
//           );

  // Disable the GPIO power-on default high-impedance mode
  // to activate previously configured port settings

  //Set P1.1 as UART Rx
  GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P1,
    GPIO_PIN1,
    UART_SELECT_FUNCTION
  );
  // Set P1.0 as UART Tx
  GPIO_setAsPeripheralModuleFunctionOutputPin(
    GPIO_PORT_P1,
    GPIO_PIN0,
    UART_SELECT_FUNCTION
  );
  PMM_unlockLPM5();
}

//------------------------------------------------------
uint8_t charCnt = 0;
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)

{


    //switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    switch(__even_in_range(UCA0IV,18))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:

            if (UCA0RXBUF !='\n')
                rx_buf[charCnt++] = UCA0RXBUF;
            else {
               // TransmitString(rx_buf);
                charCnt = 0;

            }

            break;
        case USCI_UART_UCTXIFG:
            break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG:

            break;
        default: break;
    }
}


// Initialize EUSCI
void InitEusci(void)
{
    // Configure UCA1TXD and UCA1RXD
    //P1SEL0 |= BIT0 | BIT1;

    //P1SEL1 &= ~(BIT0 | BIT1);



    //EUSCI_A_UART_enable();
    // Configure UART 9600 @ SMCLK 1 MHz
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

void ClearRxBuffer(void)
{
    unsigned char i;

    for(i = 0; i < MAX_STRBUF_SIZE; i++) {
        rx_buf[i] = 0;
    }
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


void BC95Init(void) {
    char *strx, *extstrx;
    BC95 bc95_status;
    TransmitString("AT\r\n");
    strx=strstr((const char*)rx_buf,(const char*)"OK");// Return OK
    while(strx==NULL)
        {
            ClearRxBuffer();
            //TransmitString("AT\r\n");
            TransmitString("AT\r\n");
            DelayClock(3000);
            strx=strstr((const char*)rx_buf,(const char*)"OK");
        }

    TransmitString("AT+CMEE=1\r\n"); //Acquire the error code
    DelayClock(3000);
    strx=strstr((const char*)rx_buf,(const char*)"OK");
    ClearRxBuffer();

    TransmitString("AT+NBAND?\r\n");// Acquire the frequency
    DelayClock(300);
    strx=strstr((const char*)rx_buf,(const char*)"+NBAND:5");
    ClearRxBuffer();

    while(strx==NULL)
    {
            ClearRxBuffer();
            TransmitString("AT+NBAND?\r\n");// Acquire the frequency
            DelayClock(3000);
            strx=strstr((const char*)rx_buf,(const char*)"+NBAND:5");
    }

    TransmitString("AT+CIMI\r\n");// Acquire CIMI
    DelayClock(300);
    strx=strstr((const char*)rx_buf,(const char*)"46011");//¡¤Return 46011
    ClearRxBuffer();
    while(strx==NULL)
    {
            ClearRxBuffer();
            TransmitString("AT+CIMI\r\n");// Acquire CIMI
            DelayClock(3000);
            strx=strstr((const char*)rx_buf,(const char*)"46011");// Return 46011
    }

    TransmitString("AT+CGATT=1\r\n");// Confirm the card is available or not
    DelayClock(300);
    strx=strstr((const char*)rx_buf,(const char*)"OK");//¡¤Return OK
    ClearRxBuffer();
    while(strx==NULL)
    {
            ClearRxBuffer();
            TransmitString("AT+CGATT=1\r\n");
            DelayClock(3000);
            strx=strstr((const char*)rx_buf,(const char*)"OK");
    }
/*
    TransmitString("AT+CGATT?\r\n");
    DelayClock(3000);
    strx=strstr((const char*)rx_buf,(const char*)"+CGATT:1");// Return 1
    ClearRxBuffer();
    while(strx==NULL)
    {
            ClearRxBuffer();
            TransmitString("AT+CGATT?\r\n");
            DelayClock(65000);
            strx=strstr((const char*)rx_buf,(const char*)"+CGATT:1");
    }
*/

    TransmitString("AT+CSQ\r\n");//Acquire CSQ
    DelayClock(3000);
    strx=strstr((const char*)rx_buf,(const char*)"+CSQ");//¡¤Return CSQ
    if(strx)
        {
            bc95_status.CSQ=(strx[5]-0x30)*10+(strx[6]-0x30);
            if(bc95_status.CSQ==99)// Fail to connect
            {
               // while(1)
               // {
                    // TODO
                    // Insert the code to indicate the error during the process of initializing BC95
               //     DelayClock(3000);
               // }
            }
        }
        while(strx==NULL)
        {
                ClearRxBuffer();
                TransmitString("AT+CSQ\r\n");//
                DelayClock(3000);
                strx=strstr((const char*)rx_buf,(const char*)"+CSQ");//
            if(strx)
            {
                bc95_status.CSQ=(strx[5]-0x30)*10+(strx[6]-0x30);//
                if(bc95_status.CSQ==99)//
                {
                   // while(1)
                   // {
                        // TODO
                        // Insert the code to indicate the error during the process of initializing BC95
                   //     DelayClock(3000);
                   // }
                }
            }
        }
    ClearRxBuffer();


    TransmitString("AT+CEREG?\r\n"); // Registration status
    DelayClock(3000);
    strx=strstr((const char*)rx_buf,(const char*)"+CEREG:0,1");// Return the status of registration
    extstrx=strstr((const char*)rx_buf,(const char*)"+CEREG:1,1");
    ClearRxBuffer();
    while(strx==NULL&&extstrx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+CEREG?\r\n");
        DelayClock(3000);
        strx=strstr((const char*)rx_buf,(const char*)"+CEREG:0,1");
        extstrx=strstr((const char*)rx_buf,(const char*)"+CEREG:1,1");
    }

    TransmitString("AT+CEREG=1\r\n");
    DelayClock(3000);
    strx=strstr((const char*)rx_buf,(const char*)"OK");
    ClearRxBuffer();
    while(strx==NULL&&extstrx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+CEREG=1\r\n");//
        DelayClock(3000);
        strx=strstr((const char*)rx_buf,(const char*)"OK");//
    }


}



void BC95ConnectCloud(void) {
    char *strx;

    TransmitString("AT+NRB\r\n"); // Reboot BC95
    DelayClock(300);
    strx=strstr((const char*)rx_buf,(const char*)"OK");// Return OK
    while(strx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+NRB\r\n");
        DelayClock(300);
        strx=strstr((const char*)rx_buf,(const char*)"OK");
    }

    TransmitString("AT+CFUN=0\r\n"); //Set phone function
    DelayClock(300);
    strx=strstr((const char*)rx_buf,(const char*)"OK");// Return OK
    while(strx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+CFUN=0\r\n");
        DelayClock(3000);
        strx=strstr((const char*)rx_buf,(const char*)"OK");
    }

    TransmitString("AT+NCDP=180.101.147.115,5683\r\n"); //Set and query the server IP address and port
    DelayClock(300);
    strx=strstr((const char*)rx_buf,(const char*)"OK");// Return OK
    while(strx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+NCDP=180.101.147.115,5683\r\n");
        DelayClock(3000);
        strx=strstr((const char*)rx_buf,(const char*)"OK");
    }

    TransmitString("AT+CFUN=1\r\n"); //Set phone function
    DelayClock(300);
    strx=strstr((const char*)rx_buf,(const char*)"OK");// Return OK
    while(strx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+CFUN=1\r\n");
        DelayClock(3000);
        strx=strstr((const char*)rx_buf,(const char*)"OK");
    }

    TransmitString("AT+CGDCONT=1,\"IP\",\"CTNB\"\r\n"); // Define a PDP context
    DelayClock(300);
    strx=strstr((const char*)rx_buf,(const char*)"OK");// Return OK
    while(strx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+CGDCONT=1,\"IP\",\"CTNB\"\r\n");
        DelayClock(3000);
        strx=strstr((const char*)rx_buf,(const char*)"OK");
    }

    TransmitString("AT+CGATT=1\r\n"); //Active the network
    DelayClock(300);
    strx=strstr((const char*)rx_buf,(const char*)"OK");// Return OK
    while(strx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+CGATT=1\r\n");
        DelayClock(3000);
        strx=strstr((const char*)rx_buf,(const char*)"OK");
    }
/*
    TransmitString("AT+CGPADDR\r\n"); //Show PDP address
    DelayClock(3000);
    strx=strstr((const char*)rx_buf,(const char*)"+CGPADDR:1");// Return OK
    while(strx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+CGPADDR\r\n");
        DelayClock(3000);
        strx=strstr((const char*)rx_buf,(const char*)"+CGPADDR:1");
    }*/

    DelayClock(3000);

    TransmitString("AT+NNMI=1\r\n"); // New message indication
    DelayClock(3000);
    strx=strstr((const char*)rx_buf,(const char*)"OK");// Return OK
    while(strx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+NNMI=1\r\n");
        DelayClock(3000);
        strx=strstr((const char*)rx_buf,(const char*)"OK");
    }

    DelayClock(3000);

    TransmitString("AT+NMGS=5,00012E1F63\r\n"); // Send data
    DelayClock(3000);
    strx=strstr((const char*)rx_buf,(const char*)"OK");// Return OK
    while(strx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+NMGS=5,00012E1F63\r\n");
        DelayClock(3000);
        strx=strstr((const char*)rx_buf,(const char*)"OK");
    }

    TransmitString("AT+NMGS=5,00012E1F63\r\n"); // Send data
    DelayClock(3000);
    strx=strstr((const char*)rx_buf,(const char*)"OK");// Return OK
    while(strx==NULL)
    {
        ClearRxBuffer();
        TransmitString("AT+NMGS=5,00012E1F63\r\n");
        DelayClock(3000);
        strx=strstr((const char*)rx_buf,(const char*)"OK");
    }


}





