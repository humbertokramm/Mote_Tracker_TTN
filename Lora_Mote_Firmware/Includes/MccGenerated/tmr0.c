/**
  TMR0 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr0.c

  @Summary
    This is the generated driver implementation file for the TMR0 driver using MPLAB® Code Configurator

  @Description
    This source file provides APIs for TMR0.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.10.2
        Device            :  PIC16F1509
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.33
        MPLAB             :  MPLAB X 2.26
*/

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*/

/**
  Section: Included Files
*/

#include <xc.h>
#include "tmr0.h"

/**
  Section: Global Variables Definitions
*/
volatile uint16_t timer1ReloadVal;

/**
  Section: TMR0 APIs
*/

void TMR0_Initialize(void)
{
    //Set the Timer to the options selected in the GUI

    //T0OSCEN disabled; nT0SYNC synchronize; T0CKPS 1:1; TMR0CS FOSC/4; TMR0ON disabled; 
    T0CON = 0x00;

    //T0GVAL disabled; T0GSPM disabled; T0GSS T0G; T0GTM disabled; T0GPOL low; T0GGO_nDONE done; TMR0GE disabled; 
    T0GCON = 0x00;

    //TMR0H 208;
    TMR0H = 0xD0;

    //TMR0L 00;
    TMR0L = 0x20;

    // Load the TMR value to reload variable
    timer1ReloadVal=(TMR0H << 8) | TMR0L;

    // Clearing IF flag before enabling the interrupt.
    PIR1bits.TMR0IF = 0;

    // Enabling TMR0 interrupt.
    PIE1bits.TMR0IE = 1;

    // Start TMR0
    TMR0_StartTimer();
}

void TMR0_StartTimer(void)
{
    // Start the Timer by writing to TMRxON bit
    T0CONbits.TMR0ON = 1;
}

void TMR0_StopTimer(void)
{
    // Stop the Timer by writing to TMRxON bit
    T0CONbits.TMR0ON = 0;
}

uint16_t TMR0_ReadTimer(void)
{
    uint16_t readVal;

    readVal = (TMR0H << 8) | TMR0L;

    return readVal;
}

void TMR0_WriteTimer(uint16_t timerVal)
{
    if (T0CONbits.nT0SYNC == 1)
    {
        // Stop the Timer by writing to TMRxON bit
        T0CONbits.TMR0ON = 0;

        // Write to the Timer1 register
        TMR0H = (timerVal >> 8);
        TMR0L = timerVal;

        // Start the Timer after writing to the register
        T0CONbits.TMR0ON =1;
    }
    else
    {
        // Write to the Timer1 register
        TMR0H = (timerVal >> 8);
        TMR0L = timerVal;
    }
}

void TMR0_Reload(void)
{
    //Write to the Timer1 register
    TMR0H = (timer1ReloadVal >> 8);
    TMR0L = timer1ReloadVal;
}

void TMR0_StartSinglePulseAcquisition(void)
{
    T0GCONbits.T0GGO_nDONE = 1;
}

uint8_t TMR0_CheckGateValueStatus(void)
{
    return (T0GCONbits.T0GVAL);
}

void TMR0_ISR(void)
{

    // Clear the TMR0 interrupt flag
    PIR1bits.TMR0IF = 0;

    TMR0H = (timer1ReloadVal >> 8);
    TMR0L = timer1ReloadVal;

    // Add your TMR0 interrupt custom code
}

/**
  End of File
*/
