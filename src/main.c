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
/*
// TimerA0 UpMode Configuration Parameter
Timer_A_initUpModeParam initUpParam_A0 =
{
    TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
    TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 1MHz
    1000000,                                  // 1s period
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
    TIMER_A_DO_CLEAR,                       // Clear value
    true                                    // Start Timer
};
*/
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer

    ADC_PinInit(ADC_BASE, ADC_INPUT_A3);
    ButtonInt();

    PM5CTL0 &= ~LOCKLPM5;

    InitGPIO();


    //InitClock();
    InitLCD();
    ClearLCD4463();
    ResetSetTemperature();
   // InitRTC();
    InitEusci();


//   System_Initial();

    __enable_interrupt();


    StopWatchModeInit();     // Initialize stopwatch mode

    BC95Init();
    BC95ConnectCloud();
    while (1)
    {
        IncRTC4463();

        TempSensoronChip();

        DisplaySetTemp4463( );

    }

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
void InitRTC()
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
   // GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    //GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    /*   GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
       GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
       GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
       GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
       GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
       GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
       GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);*/
    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);

   // GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
   // GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7);
    /*    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
        GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
        GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
        GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
        GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
        GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
        GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);*/
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN3);
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

    // LED (P4.0)
   // GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
   // GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);




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




