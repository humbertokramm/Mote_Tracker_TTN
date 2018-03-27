/*********************************************************************
 *
 *      Microchip LoRa Mote Example Application
 *
 *********************************************************************
 * FileName:        TMRapp.c
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18 USB microcontroller
 * Compiler:        XC8 v1.35+
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the "Company") for its PICmicro(R) Microcontroller is intended and
 * supplied to you, the Company's customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * File version         Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 1.0                  04/01/16    Original.

 ********************************************************************/
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "TMRapp.h"

static uint16_t   timerMsTickCount = 0;
static uint16_t   timerMinTickCount = 0;

void TMRapp_Tick(void)
{
    // Handle Bit flags
    appTimer.mSecTick = true;       // Single Tick
}

void TMRapp_ClearBitFlags(void)
{
    // Clear all bit Flags
    appTimer.TicksBitMap = 0x00;
}

void TMRapp_Handler(void)
{
    // Increment Counter
    timerMsTickCount++;
    // Handle Tick Increments
    if ( (timerMsTickCount % 10) == 0)
    {
        appTimer.mSecTenTick = true;
        if ( (timerMsTickCount % 100) == 0)
        {
            appTimer.mSecHundredTick = true;
            if ( (timerMsTickCount % 1000) == 0)
            {
                appTimer.SecTick = true;
                if ( (timerMsTickCount % 60000) == 0)
                {
                    timerMinTickCount++;
                    appTimer.MinTick = true;
                    if ( (timerMinTickCount % 60) == 0)
                    {
                        appTimer.HourTick = true;
                        if ( (timerMinTickCount % 720) == 0)
                        {
                            appTimer.HalfDayTick = true;
                            if ( (timerMinTickCount % 1440) == 0)
                            {
                                appTimer.DayTick = true;
                                // Reset Counters
                                timerMsTickCount = 0;
                                timerMinTickCount = 0;
                            }
                        }
                    }
                }
            }
        }
    }
}