/**
  Generated Pin Manager File

  Company:
    Microchip Technology Inc.

  File Name:
    pin_manager.c

  Summary:
    This is the Pin Manager file generated using MPLAB® Code Configurator

  Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.10
        Device            :  PIC16F1509
        Driver Version    :  1.02
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.33
        MPLAB             :  MPLAB X 2.26
*/

/*
Copyright (c) 2013 - 2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include "pin_manager.h"
#include "HardwareProfile.h"

void PIN_MANAGER_Initialize(void)
{
    //*** TRIS CONFIG *** //
    // A
    USB_DET_TRIS = INPUT;    
    VBAT_TRIS = INPUT;    
    TEMP_SENSOR_TRIS = INPUT;   
    LIGHT_SENSOR_TRIS = INPUT;
    //disable LCD backlight before powering up
    LCD_BACKLIGHT_LAT = DISABLE;
    LCD_BACKLIGHT_TRIS = OUTPUT;
    SENSE_PWR_TRIS = OUTPUT;
    LED_GREEN_TRIS = OUTPUT;   
    LED_RED_TRIS = OUTPUT; 
    // B
    SWITCH_1_TRIS = INPUT;
    SWITCH_2_TRIS = INPUT;
    LCD_RESET_TRIS = OUTPUT;
    LCD_RDN_EN_TRIS = OUTPUT;
    LCD_CS_TRIS = OUTPUT;
    MOD_RESET_TRIS = OUTPUT;
    // C
    LCD_A0_TRIS = OUTPUT;
    LCD_RW_TRIS = OUTPUT;
    MODULE_WAKE_TRIS = INPUT;
    EUSART_TX_TRIS = OUTPUT;
    EUSART_RX_TRIS = INPUT;
    // D
    TRISD = TRISD_INIT;
    // E
    LCD_BACKLIGHT_TRIS = OUTPUT;
    //*** LAT CONFIG *** //
    // A
    SENSE_PWR_LAT = PWR_ENABLE;
    LED_GREEN_LAT = LED_OFF;
    LED_RED_LAT = LED_OFF;
    // B
    LCD_RESET_LAT = DISABLE;
    // C
    MODULE_WAKE_LAT = DISABLE;
    EUSART_TX_LAT = DISABLE;
    // E
    LCD_BACKLIGHT_LAT =  ENABLE;
    //*** ANSEL CONFIG *** //
    USB_DET_ANSEL = DIGITAL;
    VBAT_ANSEL = ANALOG;
    TEMP_SENSE_ANSEL = ANALOG;
    LIGHT_SENSE_ANSEL = ANALOG;
    EUSART_RX_ANSEL = DIGITAL;
    EUSART_TX_ANSEL = DIGITAL;
}
/**
 End of File
*/