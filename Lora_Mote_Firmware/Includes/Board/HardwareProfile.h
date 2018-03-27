/*********************************************************************
 *
 *      Microchip LoRa Mote Example Application
 *
 *********************************************************************
 * FileName:        HardwareProfile.h
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
#ifndef HARDWARE_PROFILE__H
#define HARDWARE_PROFILE__H
/*******************************************************************/
/******** Application specific definitions *************************/
/*******************************************************************/

/*******************************************************************/
/******** USB stack hardware selection options *********************/
/*******************************************************************/

/*******************************************************************/
/******** System  *************************/
/*******************************************************************/
#define _XTAL_FREQ   48000000

/*******************************************************************/
/******** PORT/LAT/TRIS *************************/
/*******************************************************************/
/******** -A- *************************/
#define LATA_INIT       0b00000000  // LAT Init value
#define TRISA_INIT      0b11111111  // TRIS Init value  (0=Output)
#define ANSELA_INIT     0b00000000  // ANSEL Init value (1=Analog)
#define ODCA_INIT       0b00000000  // ODC Init value   (1=Open Drain)

/******** -0- *************************/
#define USB_DET_PORT    PORTAbits.RA0
#define USB_DET_LAT     LATAbits.LATA0
#define USB_DET_TRIS    TRISAbits.TRISA0
#define USB_DET_ANSEL   ANSELAbits.ANSA0
#define USB_DET_EN      PIE2bits.C1IE
#define USB_DET_FLAG    PIR2bits.C1IF
/******** -1- *************************/
#define VBAT_PORT   PORTAbits.RA1
#define VBAT_LAT    LATAbits.LATA1
#define VBAT_TRIS   TRISAbits.TRISA1
#define VBAT_ANSEL   ANSELAbits.ANSA1
/******** -2- *************************/
#define TEMP_SENSOR_PORT    PORTAbits.RA2
#define TEMP_SENSOR_LAT     LATAbits.LATA2
#define TEMP_SENSOR_TRIS    TRISAbits.TRISA2
#define TEMP_SENSE_ANSEL    ANSELAbits.ANSA2
/******** -3- *************************/
#define LIGHT_SENSOR_PORT   PORTAbits.RA3
#define LIGHT_SENSOR_LAT    LATAbits.LATA3
#define LIGHT_SENSOR_TRIS   TRISAbits.TRISA3
#define LIGHT_SENSE_ANSEL   ANSELAbits.ANSA3
/******** -4- *************************/
//#define VOUT_EN_PORT    PORTAbits.RA4
//#define VOUT_EN_LAT     LATAbits.LATA4
//#define VOUT_EN_TRIS    TRISAbits.TRISA4
/******** -5- *************************/
#define SENSE_PWR_PORT    PORTAbits.RA5
#define SENSE_PWR_LAT     LATAbits.LATA5
#define SENSE_PWR_TRIS    TRISAbits.TRISA5
/******** -6- *************************/
#define LED_GREEN_PORT    PORTAbits.RA6
#define LED_GREEN_LAT     LATAbits.LATA6
#define LED_GREEN_TRIS    TRISAbits.TRISA6
/******** -7- *************************/
#define LED_RED_PORT    PORTAbits.RA7
#define LED_RED_LAT     LATAbits.LATA7
#define LED_RED_TRIS    TRISAbits.TRISA7

/******** -B- *************************/
#define LATB_INIT       0b00000000  // LAT Init value
#define TRISB_INIT      0b11111111  // TRIS Init value  (0=Output)
#define ANSELB_INIT     0b00000000  // ANSEL Init value (1=Analog)
#define ODCB_INIT       0b00000000  // ODC Init value   (1=Open Drain)
/******** -0- *************************/
#define SWITCH_2_PORT    PORTBbits.RB0
#define SWITCH_2_LAT     LATBbits.LATB0
#define SWITCH_2_TRIS    TRISBbits.TRISB0
#define SWITCH_2_IOC     INTCONbits.INT0IE
/******** -1- *************************/
#define SWITCH_1_PORT    PORTBbits.RB1
#define SWITCH_1_LAT     LATBbits.LATB1
#define SWITCH_1_TRIS    TRISBbits.TRISB1
#define SWITCH_1_IOC     INTCON3bits.INT1IE
/******** -2- *************************/
#define LCD_RESET_PORT    PORTBbits.RB2
#define LCD_RESET_LAT     LATBbits.LATB2
#define LCD_RESET_TRIS    TRISBbits.TRISB2
/******** -3- *************************/
#define LCD_RDN_EN_PORT    PORTBbits.RB3
#define LCD_RDN_EN_LAT     LATBbits.LATB3
#define LCD_RDN_EN_TRIS    TRISBbits.TRISB3
/******** -4- *************************/
#define LCD_CS_PORT    PORTBbits.RB4
#define LCD_CS_LAT     LATBbits.LATB4
#define LCD_CS_TRIS    TRISBbits.TRISB4
/******** -5- *************************/
#define MOD_RESET_PORT    PORTBbits.RB5
#define MOD_RESET_LAT     LATBbits.LATB5
#define MOD_RESET_TRIS    TRISBbits.TRISB5
/******** -6- *************************/
// Not Used
/******** -7- *************************/
// Not Used

/******** -C- *************************/
#define LATC_INIT       0b00000000  // LAT Init value
#define TRISC_INIT      0b11111111  // TRIS Init value  (0=Output)
#define ANSELC_INIT     0b00000000  // ANSEL Init value (1=Analog)
#define ODCC_INIT       0b00000000  // ODC Init value   (1=Open Drain)
/******** -0- *************************/
#define LCD_A0_PORT    PORTCbits.RC0
#define LCD_A0_LAT     LATCbits.LATC0
#define LCD_A0_TRIS    TRISCbits.TRISC0
/******** -1- *************************/
#define LCD_RW_PORT    PORTCbits.RC1
#define LCD_RW_LAT     LATCbits.LATC1
#define LCD_RW_TRIS    TRISCbits.TRISC1
/******** -2- *************************/
#define MODULE_WAKE_PORT    PORTCbits.RC2
#define MODULE_WAKE_LAT     LATCbits.LATC2
#define MODULE_WAKE_TRIS    TRISCbits.TRISC2
#define MODULE_WAKE_IOC     IOCCbits.IOCC2
/******** -6- *************************/
#define EUSART_TX_PORT    PORTCbits.RC6
#define EUSART_TX_LAT     LATCbits.LATC6
#define EUSART_TX_TRIS    TRISCbits.TRISC6
#define EUSART_TX_ANSEL   ANSELCbits.ANSC6
/******** -7- *************************/
#define EUSART_RX_PORT    PORTCbits.RC7
#define EUSART_RX_LAT     LATCbits.LATC7
#define EUSART_RX_TRIS    TRISCbits.TRISC7
#define EUSART_RX_ANSEL   ANSELCbits.ANSC7

/******** -D- *************************/
#define LATD_INIT       0b00000000  // LAT Init value
#define TRISD_INIT      0b00000000  // TRIS Init value  (0=Output)
#define ANSELD_INIT     0b00000000  // ANSEL Init value (1=Analog)
#define ODCD_INIT       0b00000000  // ODC Init value   (1=Open Drain)
/******** -0- *************************/
#define LCD_D0_PORT    PORTDbits.RD0
#define LCD_D0_TRIS    TRISDbits.TRISD0
#define LCD_D0_LAT     LATDbits.LATD0
/******** -1- *************************/
#define LCD_D1_PORT    PORTDbits.RD1
#define LCD_D1_TRIS    TRISDbits.TRISD1
#define LCD_D1_LAT     LATDbits.LATD1
/******** -2- *************************/
#define LCD_D2_PORT    PORTDbits.RD2
#define LCD_D2_TRIS    TRISDbits.TRISD2
#define LCD_D2_LAT     LATDbits.LATD2
/******** -3- *************************/
#define LCD_D3_PORT    PORTDbits.RD3
#define LCD_D3_TRIS    TRISDbits.TRISD3
#define LCD_D3_LAT     LATDbits.LATD3
/******** -4- *************************/
#define LCD_D4_PORT    PORTDbits.RD4
#define LCD_D4_TRIS    TRISDbits.TRISD4
#define LCD_D4_LAT     LATDbits.LATD4
/******** -5- *************************/
#define LCD_D5_PORT    PORTDbits.RD5
#define LCD_D5_TRIS    TRISDbits.TRISD5
#define LCD_D5_LAT     LATDbits.LATD5
/******** -6- *************************/
#define LCD_D6_PORT    PORTDbits.RD6
#define LCD_D6_TRIS    TRISDbits.TRISD6
#define LCD_D6_LAT     LATDbits.LATD6
/******** -7- *************************/
#define LCD_D7_PORT    PORTDbits.RD7
#define LCD_D7_TRIS    TRISDbits.TRISD7
#define LCD_D7_LAT     LATDbits.LATD7

/******** -E- *************************/
#define LATE_INIT       0b00000000  // LAT Init value
#define TRISE_INIT      0b11111111  // TRIS Init value  (0=Output)
#define ANSELE_INIT     0b00000000  // ANSEL Init value (1=Analog)
#define ODCE_INIT       0b00000000  // ODC Init value   (1=Open Drain)
/******** -0- *************************/
// Not Used
/******** -1- *************************/
// Not Used
/******** -2- *************************/
#define LCD_BACKLIGHT_PORT    PORTEbits.RE2
#define LCD_BACKLIGHT_TRIS    TRISEbits.TRISE2
#define LCD_BACKLIGHT_LAT     LATEbits.LATE2
/******** -3- *************************/
// Not Used
// END OF PORTS
/*******************************************************************/
/******** Peripheral  *************************/
/*******************************************************************/
#define IOC_ENABLE  INTCONbits.IOCIE
#define IOC_FLAG    INTCONbits.IOCIF
#define INT0_FLAG    INTCONbits.INT0IF
#define INT1_FLAG    INTCON3bits.INT1IF

/*******************************************************************/
/******** General  *************************/
/*******************************************************************/
#define LED_ON  1
#define LED_OFF 0

#define ENABLE  1
#define DISABLE 0

#define PWR_ENABLE 0
#define PWR_DISABLE 1

#define INPUT   1
#define OUTPUT  0

#define ANALOG  1
#define DIGITAL 0

#endif  //HARDWARE_PROFILE__H
