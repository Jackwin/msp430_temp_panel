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
#include "uart.h"
#include "bc95.h"


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




