/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 *
 * hal_LCD.c
 *
 * Hardware abstraction layer for the FH-1138P Segmented LCD
 *
 * September 2014
 * E. Chen
 *
 ******************************************************************************/

#include <spi_intf.h>
//#include "main.h"



void Init_spi()
{
//    // P1.1 - UCA0SOMI
//    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION);
//    // P1.0 - UCA0SIMO
//    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);
//    // P1.2 - UCA0CLK
//    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);
    // P1.3 - UCA0STE
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3);
    // P1.5 - RF_SDN
//    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);

    // Configure SPI Pins for UCA0CLK, UCA0TXD/UCA0SIMO and UCA0RXD/UCA0SOMI
    /*
     * Select Port 1
     * Set Pin 0, Pin 1 and Pin 2 to input Secondary Module Function
     */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P1,
        GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2,
        GPIO_PRIMARY_MODULE_FUNCTION
        );

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    EUSCI_A_SPI_initMasterParam initParams = {0};
    initParams.clockPhase = EUSCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    initParams.clockPolarity = EUSCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    initParams.clockSourceFrequency = 2000000;
    initParams.desiredSpiClock = 1000000;
    initParams.msbFirst = EUSCI_A_SPI_MSB_FIRST;
    initParams.selectClockSource = EUSCI_A_SPI_CLOCKSOURCE_SMCLK; // SMCLK default is 2MHz
    initParams.spiMode = EUSCI_A_SPI_3PIN;

    // Init spi as 3-pin mode
    EUSCI_A_SPI_initMaster(EUSCI_A0_BASE, &initParams);

    // Enable SPI
    EUSCI_A_SPI_enable(EUSCI_A0_BASE);
}


void Init_spi_B()
{
//    // P1.1 - UCA0SOMI
//    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_SECONDARY_MODULE_FUNCTION);
//    // P1.0 - UCA0SIMO
//    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_SECONDARY_MODULE_FUNCTION);
//    // P1.2 - UCA0CLK
//    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);
    // P1.3 - UCA0STE
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);
    // P1.5 - RF_SDN
//    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);

    // Configure SPI Pins for UCA0CLK, UCA0TXD/UCA0SIMO and UCA0RXD/UCA0SOMI
    /*
     * Select Port 1
     * Set Pin 0, Pin 1 and Pin 2 to input Secondary Module Function
     */
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_P5,
        GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3,
        GPIO_PRIMARY_MODULE_FUNCTION
        );

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    EUSCI_B_SPI_initMasterParam initParams = {0};
    initParams.clockPhase = EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
    initParams.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
    initParams.clockSourceFrequency = 2000000;
    initParams.desiredSpiClock = 1000000;
    initParams.msbFirst = EUSCI_B_SPI_MSB_FIRST;
    initParams.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK; // SMCLK default is 2MHz
    initParams.spiMode = EUSCI_B_SPI_3PIN;

    // Init spi as 3-pin mode
    EUSCI_B_SPI_initMaster(EUSCI_B0_BASE, &initParams);

    // Enable SPI
    EUSCI_B_SPI_enable(EUSCI_B0_BASE);
}

/*===========================================================================
* 函数 ：SPI_ExchangeByte() => 通过SPI进行数据交换                          *
* 输入 ：需要写入SPI的值                                                    *
* 输出 ：通过SPI读出的值                                                    *
============================================================================*/
INT8U SPI_ExchangeByte(INT8U input)
{
//    //USCI_A0 TX buffer ready?
//    while(!EUSCI_A_SPI_getInterruptStatus(EUSCI_A0_BASE,
//                                          EUSCI_A_SPI_TRANSMIT_INTERRUPT))
//    {
//        ;
//    }

    //Transmit Data to slave
    EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, input);
    //USCI_B0 TX buffer ready?
    while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
                                          EUSCI_B_SPI_TRANSMIT_INTERRUPT))
    {
        ;
    }
    //USCI_B0 RX buffer ready?
    while(!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
                                          EUSCI_B_SPI_RECEIVE_INTERRUPT))
    {
        ;
    }

    // not necessary as of wait time above
    //__delay_cycles(10);  // about 9us. The min hold time of cs is 50ns
    return EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);
}

/*
INT8U SPI_ExchangeByte(INT8U input)
{
//    //USCI_A0 TX buffer ready?
//    while(!EUSCI_A_SPI_getInterruptStatus(EUSCI_A0_BASE,
//                                          EUSCI_A_SPI_TRANSMIT_INTERRUPT))
//    {
//        ;
//    }

    //Transmit Data to slave
    EUSCI_A_SPI_transmitData(EUSCI_A0_BASE, input);
    //USCI_A0 TX buffer ready?
    while(!EUSCI_A_SPI_getInterruptStatus(EUSCI_A0_BASE,
                                          EUSCI_A_SPI_TRANSMIT_INTERRUPT))
    {
        ;
    }
    //USCI_A0 RX buffer ready?
    while(!EUSCI_A_SPI_getInterruptStatus(EUSCI_A0_BASE,
                                          EUSCI_A_SPI_RECEIVE_INTERRUPT))
    {
        ;
    }

    // not necessary as of wait time above
    //__delay_cycles(10);  // about 9us. The min hold time of cs is 50ns
    return EUSCI_A_SPI_receiveData(EUSCI_A0_BASE);
}*/


