/*********************************************************************
 *
 *      Microchip LoRa Mote Example Application
 *
 *********************************************************************
 * FileName:        main.c
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

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "mcc.h"
#include "memory.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_cdc.h"
#include "SSD1306oLED.h"
#include "MOTEapp.h"
#include "USBapp.h"
#include "HardwareProfile.h"

// Application Types
typedef enum {USB, MOTE} OPERATION_T;
OPERATION_T operationType;
typedef enum {MAIN_STARTUP, MAIN_CDC_MODE, MAIN_HOST_MODE} MAIN_STATE_T;
MAIN_STATE_T mainState;
typedef enum {APP_LORA_STARTUP, APP_HOST_RUNNING} APP_STATE_T;
APP_STATE_T appState;
// Application Variables
MOTE_MODE_T moteModeState;
// Local Functions PROTOTYPES

// Local Functions PROTOTYPES
static uint8_t powerOnStatus = 0x00;
static uint8_t dataRateRead = 0x00;

// TODO: poweruptest.c needs a header for this prototype
extern void test_powerup(void);

// Application Start
void main(void)
{
    // Prepare Application; MCC generated code
    SYSTEM_Initialize();
    // Custom Initializations
    oled_init();         // Dispaly
    USBDeviceInit();	// usb_device.c.  Initializes USB module SFRs and firmware
                        // variables to known states.
    USBDeviceAttach();
    // MCLR RN Module
    MOD_RESET_LAT = ENABLE;
    moteApp_delayms(50);
    MOD_RESET_LAT = DISABLE;
    moteApp_delayms(500);
    // Interrupts Enabled
    PEIE = 1;
    GIE = 1;
    // Test Hardware
    moduleResync();
    moteApp_delayms(20);
    moteApp_clearCommand();
    moteApp_delayms(20);
    
    powerOnStatus = DATAEE_ReadByte(0x00);
    if ( powerOnStatus != 0x55)
    {
        test_powerup();
        DATAEE_WriteByte(0x00, 0x55);
    }

    // Default to USB
    operationType = USB;   // Mode State
    USBapp_handlerState(USB_STARTUP);
    // Initial Display Print
    oled_clear();
    oled_putString("Microchip Mote",0,0);
    oled_putString("   LoRa Demo  ",0,1);
    oled_putString(" V 1.0.1 Code ",0,3);
    moteApp_delayms(1000);
   
    // Application Loop
    while(1)
    {
        // Handle USB or Solo Mode
        switch (operationType)
        {
            default:        // Invalid
                while(1);   // Hold For Error;
            break;
            case USB:
                USBapp_Handler();   // It is held in here.
                // USB escaped; Cable is unplugged; Change OPERATION mode.
                operationType = MOTE;
                MOTEapp_handlerState(MOTE_STARTUP);   // Initial state for startup
            break;
            case MOTE:
                if (moteHandler() == MOTE_SWAP)
                {
                    // Return to USB Operation
                    operationType = USB;   // Mode State
                    USBapp_handlerState(USB_STARTUP);
                }
            break;
        }
    }
}