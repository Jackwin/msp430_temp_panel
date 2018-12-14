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

#include "hal_LCD_4463.h"
#include "string.h"
#include "main.h"
#include "lcd_e.h"

//32 pin LCD with newboard, numberic table for 1,2,3,4,5,6 LED
const char digit4463_1_6[10] =
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

//32 pin LCD with newboard, numberic table for 7,8,9,10 LED
const char digit4463_7_10[10] =
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

// LCD memory map for uppercase letters
const char alphabetBig[26][2] =
{
    {0xEF, 0x00},  /* "A" LCD segments a+b+c+e+f+g+m */
    {0xF1, 0x50},  /* "B" */
    {0x9C, 0x00},  /* "C" */
    {0xF0, 0x50},  /* "D" */
    {0x9F, 0x00},  /* "E" */
    {0x8F, 0x00},  /* "F" */
    {0xBD, 0x00},  /* "G" */
    {0x6F, 0x00},  /* "H" */
    {0x90, 0x50},  /* "I" */
    {0x78, 0x00},  /* "J" */
    {0x0E, 0x22},  /* "K" */
    {0x1C, 0x00},  /* "L" */
    {0x6C, 0xA0},  /* "M" */
    {0x6C, 0x82},  /* "N" */
    {0xFC, 0x00},  /* "O" */
    {0xCF, 0x00},  /* "P" */
    {0xFC, 0x02},  /* "Q" */
    {0xCF, 0x02},  /* "R" */
    {0xB7, 0x00},  /* "S" */
    {0x80, 0x50},  /* "T" */
    {0x7C, 0x00},  /* "U" */
    {0x0C, 0x28},  /* "V" */
    {0x6C, 0x0A},  /* "W" */
    {0x00, 0xAA},  /* "X" */
    {0x00, 0xB0},  /* "Y" */
    {0x90, 0x28}   /* "Z" */
};

void InitLCD()
{
    // L0~L27 & L36~L39 pins selected
    LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_SEGMENT_LINE_27);
    LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_36, LCD_E_SEGMENT_LINE_39);

    LCD_E_initParam initParams = LCD_E_INIT_PARAM;
    initParams.clockDivider = LCD_E_CLOCKDIVIDER_3;
    initParams.muxRate = LCD_E_4_MUX;
    initParams.segments = LCD_E_SEGMENTS_ENABLED;

    // Init LCD as 4-mux mode
    LCD_E_init(LCD_E_BASE, &initParams);

    // LCD Operation - Mode 3, internal 3.02v, charge pump 256Hz
    LCD_E_setVLCDSource(LCD_E_BASE, LCD_E_INTERNAL_REFERENCE_VOLTAGE, LCD_E_EXTERNAL_SUPPLY_VOLTAGE);
    LCD_E_setVLCDVoltage(LCD_E_BASE, LCD_E_REFERENCE_VOLTAGE_2_96V);

    LCD_E_enableChargePump(LCD_E_BASE);
    LCD_E_setChargePumpFreq(LCD_E_BASE, LCD_E_CHARGEPUMP_FREQ_16);

    // Clear LCD memory
    LCD_E_clearAllMemory(LCD_E_BASE);

    // Configure COMs and SEGs
    // L39 = COM0, L38 = COM1, L37 = COM2, L36 = COM3
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_39, LCD_E_MEMORY_COM0);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_38, LCD_E_MEMORY_COM1);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_37, LCD_E_MEMORY_COM2);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_36, LCD_E_MEMORY_COM3);

    // Select to display main LCD memory
    LCD_E_selectDisplayMemory(LCD_E_BASE, LCD_E_DISPLAYSOURCE_MEMORY);

    // Turn on LCD
    LCD_E_on(LCD_E_BASE);
}

/*
 * Scrolls input string across LCD screen from left to right
 */
void DisplayScrollText(char *msg)
{
    int length = strlen(msg);
    int oldmode = *mode;
    int i;
    int s = 5;
    char buffer[6] = "      ";
    for (i=0; i<length+7; i++)
    {
        if (*mode != oldmode)
            break;
        int t;
        for (t=0; t<6; t++)
            buffer[t] = ' ';
        int j;
        for (j=0; j<length; j++)
        {
            if (((s+j) >= 0) && ((s+j) < 6))
                buffer[s+j] = msg[j];
        }
        s--;

        showChar(buffer[0], pos1);
        showChar(buffer[1], pos2);
        showChar(buffer[2], pos3);
        showChar(buffer[3], pos4);
        showChar(buffer[4], pos5);
        showChar(buffer[5], pos6);

        __delay_cycles(200000);
    }
}

/*
 * Displays input character at given LCD digit/position
 * Only spaces, numeric digits, and uppercase letters are accepted characters
 */
void ShowChar4463(char c, int position)
{
    if (c >= '0' && c <= '9')
    {
        // Display digit
       LCDMEM[position] = digit4463_7_10[c-48]|0x08;
     //  LCDMEM[position] = 0x9e | 0x01;//digit4463[3]|0x01;
    }
    else
    {
        // Turn all segments on if character is not a space, digit, or uppercase letter
        LCDMEM[position] = 0xFF|0x08;
    }
}


/*
 * Displays input character at given LCD digit/position for 4463 Time LCD
 * */

void TimeShowChar4463(char c, int position)
{
    if (c >= '0' && c <= '9')
    {
        // Display digit
       LCDMEM[position] = digit4463_1_6[c-48]|0x01;
     //  LCDMEM[position] = 0x9e | 0x01;//digit4463[3]|0x01;
    }
    else
    {
        // Turn all segments on if character is not a space, digit, or uppercase letter
        LCDMEM[position] = 0xFF|0x01;
    }
}

/*
 * Displays input character at given LCD digit/position for 4463 temperature LCD
 * */

void TemperatureShowChar4463(char c, int position)
{
    if (c >= '0' && c <= '9')
    {
        // Display digit
       LCDMEM[position] = digit4463_7_10[c-48]|0x08;
     //  LCDMEM[position] = 0x9e | 0x01;//digit4463[3]|0x01;
    }
    else
    {
        // Turn all segments on if character is not a space, digit, or uppercase letter
        LCDMEM[position] = 0xFF|0x08;
    }
}


/*
 * Clears memories to all 6 digits on the LCD
 */
void ClearLCD()
{
    LCDMEMW[pos1/2] = 0;
    LCDMEMW[pos2/2] = 0;
    LCDMEMW[pos3/2] = 0;
    LCDMEMW[pos4/2] = 0;
    LCDMEMW[pos5/2] = 0;
    LCDMEMW[pos6/2] = 0;
    LCDMEM[12] = LCDMEM[13] = 0;
}

void ClearLCD4463()
{
       LCDMEM[0] = 0x00;
       LCDMEM[1] = 0x00;
       LCDMEM[2] = 0x00;
       LCDMEM[3] = 0x00;
       LCDMEM[4] = 0x00;//0x00;
       LCDMEM[5] = 0x00;//0x0;
       LCDMEM[6] = 0x00;//0x0;
       LCDMEM[7] = 0x00;//0x0;
       LCDMEM[8] = 0x00;//0x0;
       LCDMEM[9] = 0x00;//0x0;
       LCDMEM[10] = 0x00;//0x0;
       LCDMEM[11] = 0x00;//0x0;
       LCDMEM[12] = 0x00;//0x0;
       LCDMEM[13] = 0x00;//0x0;
}
