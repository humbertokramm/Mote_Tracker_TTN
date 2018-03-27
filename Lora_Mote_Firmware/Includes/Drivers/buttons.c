/*********************************************************************
 *
 *      Microchip LoRa Mote Example Application
 *
 *********************************************************************
 * FileName:        buttons.c
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

#include "buttons.h"
#include "HardwareProfile.h"

#define PUSH_BUTTON_SHORT   55
#define PUSH_BUTTON_RELEASE  20

/* Service PushButton
 * Called at a 1mS rate.  Times the length of a button press and sets the appropriate bit.
 */
buttonStates_t  service_pushbutton()
{
    static uint32_t  PB_Timer = 0;
    static uint8_t   Release_Timer = 0;
    buttonStates_t bs = NoButton;

    if ((SWITCH_1_PORT == 0) || (SWITCH_2_PORT == 0) ) // pushed
    {  // run 1mS timer when pushbutton is pressed
        ++PB_Timer;
        
        // Determine if press was invalid (switch bounce), a short press or a long press.
            // Check for jitter
            if (PB_Timer == PUSH_BUTTON_SHORT)
            {
                Release_Timer = 0;
                // short button press detected
                if (SWITCH_1_PORT == 0)
                {
                    bs = ShortPress1;
                }
                else
                {
                    bs = ShortPress2;
                }
            }
            else
            {
                bs = NoButton;
            }
    }
    else // released
    {
        if (Release_Timer == PUSH_BUTTON_RELEASE)
        {
            PB_Timer = 0;
            Release_Timer = 0;
        }
        else
        {
            Release_Timer++;
        }
        bs = Released;
    }

    return bs;
}
