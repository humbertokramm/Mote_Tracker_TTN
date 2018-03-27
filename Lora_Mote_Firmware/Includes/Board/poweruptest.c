/*********************************************************************
 *
 *      Microchip LoRa Mote Example Application
 *
 *********************************************************************
 * FileName:        poweruptest.c
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
#include "mcc.h"
#include "SSD1306oLED.h"
#include "MOTEapp.h"
#include "adc.h"

void test_oled(void);
void test_switch(void);
void test_comms(void);
void test_leds(void);
void test_rf(void);
void test_adc(void);
void delayms(uint16_t t);

#define VERSION_MAJOR "0"
#define VERSION_MINOR "1"

void delayms(uint16_t t)
{
    uint16_t i;
    for(i = 0; i < t; i++)
    {
        __delay_ms(1);
    }
}
void test_powerup(void)
{
    // OLED TEST
    test_oled();
    moteApp_delayms(20);
    // SWITCH TEST
    test_switch();
    // COMMS TEST
    oled_clear();
    moteApp_delayms(20);
    test_comms();
    moteApp_delayms(20);
    // LED BLINK TEST
    test_leds();
    // TEST RF COMMS
//    test_adc();
    moteApp_delayms(20);
    EUSART_FlushBuffer();
    moteApp_delayms(20);
    moteApp_testSensors();
    moteApp_delayms(3000);
 //   test_rf();
}

void test_oled(void)
{
    oled_clear();
    // Power Up Display
    oled_putString("MoTe v .",0,0);
    oled_putString(VERSION_MAJOR,0,6);
    oled_putString(VERSION_MINOR,0,8);
    oled_putString("Build Date/Time",0,1);
    oled_putString(__DATE__,0,2);
    oled_putString(__TIME__,0,3);

}

void test_switch(void)
{
    uint16_t timeout = 500;
    uint8_t  buttonCheck = 0x00;
    oled_clear();
    oled_putString("Button Test:",0,0);

    if( (SWITCH_1_PORT == 0) || (SWITCH_2_PORT == 0) )
    {
        oled_putString("Button FAIL",0,3);
    }
    else
    {
        oled_putString("Press Button",0,1);
    }
    while(timeout--)
    {
        if(SWITCH_1_PORT == 0)
        {
            oled_putString("Button1 PASS",0,2);
            buttonCheck = buttonCheck + 0x08;
        }
        if(SWITCH_2_PORT == 0)
        {
            oled_putString("Button2 PASS",0,3);
            buttonCheck = buttonCheck + 0x80;
        }
        if (buttonCheck == 0x88)
        {   // Both Button Passed; Move on
            break;
        }
        __delay_ms(10);
    }
}

void test_comms(void)
{
    oled_putString("Comm Test:",0,0);
    moteApp_toggleLeds(true);
    delayms(500);
    moteApp_toggleLeds(false);
    delayms(500);
    moteApp_toggleLeds(true);
    delayms(500);
    moteApp_toggleLeds(false);
    delayms(500);
}

void test_leds(void)
{    
    oled_putString("LED Test:",0,1);
//    moteApp_toggleLeds(true);
    LED_GREEN_LAT = 1;
    LED_RED_LAT = 1;
    delayms(500);
    LED_GREEN_LAT = 0;
    LED_RED_LAT = 0;
    delayms(500);
    LED_GREEN_LAT = 1;
    LED_RED_LAT = 1;
    delayms(500);
    LED_GREEN_LAT = 0;
    LED_RED_LAT = 0;
    delayms(500);
}

void test_adc(void)
{
    uint16_t L,Traw,F;
    uint8_t count, T;

    oled_clear();
    oled_putString("ADC Test",0,0);
    oled_putString("Temp  =",0,1);
    oled_putChar('C',14,1);
    oled_putString("Light =",0,2);

    for(count=0;count< 50;count++)
    {
//        ADC_StartConversion(FVR_channel);
        __delay_ms(16);
        __delay_ms(16);
        __delay_ms(16);
        __delay_ms(16);
        Traw = ADC_GetConversion(Temp_channel);
        T = ADC_TempConversion(Traw);

        __delay_ms(16);
        __delay_ms(16);
        __delay_ms(16);
        __delay_ms(16);
        L = ADC_GetConversion(Light_channel);
      
        oled_putUint8(T,10,1);
        oled_putUint16(L,10,2);
    }
}

void test_rf(void)
{
    oled_clear();
    oled_putString("RF Test:",0,0);
    __delay_ms(250);
}



