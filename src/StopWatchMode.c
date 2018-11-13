/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
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
 * StopWatchMode.c
 *
 * Simple stopwatch application that supports counting up and split time.
 *
 * February 2015
 * E. Chen
 *
 ******************************************************************************/

#include "StopWatchMode.h"
#include "hal_LCD_4463.h"
#include "main.h"

// Backup Memory variables to track states through LPM3.5

volatile unsigned char * Centiseconds = &BAKMEM0_L;     // Centiseconds
volatile unsigned char * Seconds = &BAKMEM0_H;          // Seconds
volatile unsigned char * Minutes = &BAKMEM1_L;          // Minutes
volatile unsigned char * Hours = &BAKMEM1_H;            // Hours
volatile unsigned char * count = &BAKMEM7_H;            // Count variable
volatile unsigned char * Weeks = &BAKMEM7_L;
volatile unsigned char * Seconds1 = &BAKMEM2_L;
volatile unsigned char * Seconds2 = &BAKMEM2_H;

void minutesinc()
{
    (*Minutes)++;
    (*Minutes) %= 60;
}

void hoursinc()
{
    (*Hours)++;
    (*Hours) %= 24;

    if((*Hours)== 0)
    {
      (*Weeks)++;
      (*Weeks) %= 7;
    }
}

void stopWatch()
{
    while(*stopWatchRunning)
    {
        // stays in LPM3 while stopwatch is running and wakes up every 10ms to update clock and LCD
        __bis_SR_register(LPM3_bits | GIE);         // Enter LPM3
        __no_operation();

        if (*stopWatchRunning)
        	// Increment clock by 10 ms and update display
            Inc_RTC();
    }

    // Loop in LPM3 to while buttons are held down and debounce timer is running
    while(TA0CTL & MC__UP)
    {
        __bis_SR_register(LPM3_bits | GIE);         // Enter LPM3
        __no_operation();
    }

    if (*mode == STOPWATCH_MODE)
    {
        PMM_turnOffRegulator();
        __bis_SR_register(LPM4_bits | GIE);         // re-enter LPM3.5
        __no_operation();
    }
}

void stopWatchModeInit()
{
    //*stopWatchRunning = 0;
    //LCDMEMCTL &= ~LCDDISP;
//    *mode = STOPWATCH_MODE ;
    // Set RTC modulo to 327-1 to trigger interrupt every ~10 ms
    RTC_stop(RTC_BASE);
//    RTC_setModulo(RTC_BASE, 326);
    resetStopWatch();
    Init_RTC();
    //displayScrollText("STOPWATCH MODE");

    // Clear stopwatch


    //*S1buttonDebounce = 0;
    //*S2buttonDebounce = 0;

    // Check if any button is pressed
//    Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
    //__bis_SR_register(LPM3_bits | GIE);         // enter LPM3

    // Disable ADC, TimerA1, Internal Ref and Temp used by TempSensor Mode
   // ADC_disable(ADC_BASE);
   // ADC_disableConversions(ADC_BASE,ADC_COMPLETECONVERSION);

//    Timer_A_stop(TIMER_A1_BASE);

   // PMM_disableInternalReference();
   // PMM_disableTempSensor();
   // PMM_turnOffRegulator();

  //  __bis_SR_register(LPM4_bits | GIE);         // enter LPM3.5
}

// Increment Real Time Counter
/*
void Inc_RTC()
{
    // Clock increment logic
    // Handles maximum 100 hours, then wraps over to 00:00:00
    (*Centiseconds)++;
    (*Centiseconds) %= 100;
    if ((*Centiseconds) == 0)
    {
        (*Seconds)++;
        (*Seconds) %= 60;
        if ((*Seconds) == 0)
        {
            (*Minutes)++;
            (*Minutes) %= 60;
            if ((*Minutes) == 0)
            {
                ++(*Hours);
                (*Hours) %= 100;
            }
        }
    }

    // Update LCD with new time
    displayTime4463();
}
*/
// Increment Real Time Counter
void Inc_RTC4463()
{
    // Clock increment logic
    // Handles maximum 100 hours, then wraps over to 00:00:00
  //  (*Centiseconds)++;
  //  (*Centiseconds) %= 100;
//    Seconds2 = Seconds1;
//    Seconds1 = Seconds;

  //  if ((*Centiseconds) == 0)
  //  {
        //(*Seconds)=(*count);
 /*       (*Seconds) %= 60;

   if(Seconds2 != Seconds)
   {
        if ((*Seconds) == 0)
        {
            (*Minutes)++;
            (*Minutes) %= 60;
            if ((*Minutes) == 0)
            {
                ++(*Hours);
                (*Hours) %= 24;
                if((*Hours)== 0)
                 {
                   (*Weeks)++;
                   (*Weeks) %= 7;
                 }
            }
        }
   }*/
   // }

    // Update LCD with new time
    displayTime4463();
}

void resetStopWatch()
{
    //*Centiseconds = 0;
    *Seconds = 50;
    *Minutes = 59;
    *Hours = 23;
    *count = 0;
    *Weeks = 6;
    // Update LCD with new time
    displayTime4463();
}


void displayTime4463()
{
    // Display Minute, Second, Centiseconds if below 1 hour mark.
 //   if ((*Hours) == 0)
  //  {
       // Time_showChar4463((*Centiseconds) % 10 + '0',pos6);
       // Time_showChar4463((*Centiseconds) / 10 + '0',pos5);
 //       Time_showChar4463((*Seconds) % 10 + '0',4);
  //      Time_showChar4463((*Seconds) / 10 + '0',3);
  //      Time_showChar4463((*Minutes) % 10 + '0',2);
  //      Time_showChar4463((*Minutes) / 10 + '0',1);
  //  }
    // Otherwise, display Hour, Minute, Second
 //   else
  //  {
   //     Time_showChar4463((*Seconds) % 10 + '0',4);
   //     Time_showChar4463((*Seconds) / 10 + '0',3);
   //     Time_showChar4463((*Minutes) % 10 + '0',2);
   //     Time_showChar4463((*Minutes) / 10 + '0',1);


        Time_showChar4463((*Minutes) % 10 + '0',4);
        Time_showChar4463((*Minutes) / 10 + '0',3);
        Time_showChar4463((*Hours) % 10 + '0',2);
        Time_showChar4463((*Hours) / 10 + '0',1);
        Time_showChar4463(((*Weeks)+1) + '0',6);
      //  Time_showChar4463(0x00,6);
      //  LCDMEM[6] = 0x01;//0x0;
        LCDMEM[0] = 0xff;
  //  }



// Workaround LCDBMEM definition bug in IAR header file
/*
#ifdef __IAR_SYSTEMS_ICC__
    if ((*Centiseconds) == 0)
    {
        LCDMEM[12] |= 0x08;
        LCDBM12 |= 0x08;
    }
    if ((*Centiseconds) == 50)
    {
        LCDMEM[12] &= ~0x08;
        LCDBM12 &= ~0x08;
    }

    // Display the 2 colons
    LCDMEM[7] |= 0x04;
    LCDM39 |= 0x04;

    LCDMEM[11] |= 0x04;
    LCDBM11 |= 0x04;
#else    
    // Blink Stopwatch symbol
    if ((*Centiseconds) == 0)
    {
        LCDMEM[12] |= 0x08;
        LCDBMEM[12] |= 0x08;
    }
    if ((*Centiseconds) == 50)
    {
        LCDMEM[12] &= ~0x08;
        LCDBMEM[12] &= ~0x08;
    }

    // Display the 2 colons
    LCDMEM[7] |= 0x04;
    LCDBMEM[7] |= 0x04;
    LCDMEM[11] |= 0x04;
    LCDBMEM[11] |= 0x04;
#endif
*/

}


/*
 * RTC Interrupt Service Routine
 * Wakes up every ~10 milliseconds to update stopwatch
 */

#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR(void)
{
    switch(__even_in_range(RTCIV, RTCIV_RTCIF))
    {
        case RTCIV_NONE : break;
        case RTCIV_RTCIF:
    //        if (*mode == STARTUP_MODE)
    //        {
    //            __bic_SR_register_on_exit(LPM3_bits);            // exit LPM3
    //        }
     //       if (*mode == STOPWATCH_MODE)
     //       {
                // Since RTC runs at 32768 Hz and isn't fast enough to count 10 ms exactly
                // offset RTC counter every 100 10ms intervals to add up to 1s
                // (327 * 32) + (328 * 68) = 32768
             /*   if((*count)==31)
                {
                    // Set RTC to interrupt after 328 XT1 cycles
                    RTC_setModulo(RTC_BASE, 327);
                }
                else if((*count)==99)
                {
                    // Set RTC to interrupt after 327 XT1 cycles
                    RTC_setModulo(RTC_BASE, 326);
                    (*count)=0;
                    (*Seconds)++;
                }
                (*count)++;  */

                 (*Seconds)++;

                 (*Seconds) %= 60;
                  if ((*Seconds) == 0)
                  {
                      (*Minutes)++;
                      (*Minutes) %= 60;
                      if ((*Minutes) == 0)
                      {
                          (*Hours)++;
                          (*Hours) %= 24;

                          if((*Hours)== 0)
                           {
                             (*Weeks)++;
                             (*Weeks) %= 7;
                           }
                      }
                  }

      //          __bic_SR_register_on_exit(LPM3_bits);            // exit LPM3
       //     }
       //     if (*mode == TEMPSENSOR_MODE)
      //          __bic_SR_register_on_exit(LPM3_bits);            // exit LPM3
            break;
    }
}




