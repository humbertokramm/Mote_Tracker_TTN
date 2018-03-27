/*********************************************************************
 *
 *      Microchip LoRa Mote Example Application
 *
 *********************************************************************
 * FileName:        USBapp.c
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

/** INCLUDES *******************************************************/
#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
// Application Includes
#include "USBapp.h"
#include "MOTEapp.h"
#include "SSD1306oLED.h"
// USB Library
#include "usb.h"
#include "usb_config.h"
#include "usb_device_cdc.h"

// Communication Includes
#include "eusart.h"
#include "HardwareProfile.h"

static USB_MODE_T usbState = USB_STARTUP;
static uint8_t USB_Buffer[CDC_DATA_OUT_EP_SIZE];
static volatile uint8_t rxByteFlag = 0;

uint8_t  NextUSBOut;
bool RS232_Out_Data_Rdy = 0;
uint8_t    LastRS232Out;  // Number of characters in the buffer
uint8_t    RS232cp;       // current position within the buffer

#define mDataRdyUSART() USBapp_getrxByteFlag()
#define mTxRdyUSART()   TXSTAbits.TRMT

uint8_t USBapp_getUsbBuffer(uint8_t index)
{
    return USB_Buffer[index];
}

uint8_t USBapp_getBufferLength(void)
{
    return LastRS232Out;
}

void USBapp_handlerState(USB_MODE_T state)
{
    usbState = state;
}


EXIT_USB_T exitUSB = CDC_USB;

void USBapp_Handler(void)
{
    // This will block in USB Mode.
    do
    {       
        switch(usbState)
        {
            default:        // Invalid
                while(1);   // Hold For Error;
            break;
            case USB_STARTUP:
                // Display
                oled_clear();
                oled_putString("CDC Mode",1,0);
                oled_putString("Command",2,1);
                oled_putString("USB Handler",0,2);
                // Change Hanlder State
                usbState = USB_RUNNING;
            break;
            case USB_RUNNING:
                // Wait/Confirm USB is already in a working configured state
                if( USBGetDeviceState() < CONFIGURED_STATE )
                {
                    /* Not ready; Jump back to the top of the while loop. */
                    continue;
                }
                usbState = USB_CDC;
            break;
            case USB_CDC:
                // USB Ready; Process USB Actions
                USBapp_DeviceCdcEmulatorHandler();
                // Application Handler during USB operation
                //  <---- Add Call here if applicable.
                // Handle button press events for menu navigation
                buttonStates_t buttonPress = Released;
                buttonPress = service_pushbutton();
                if (buttonPress != Released )
                {
                    exitUSB = USB_BAT;
                }
            break;
        }
    }while((USB_DET_PORT) && (exitUSB == NO_USB)); // USB has been Disconnected; Escape
    NOP();
    if (exitUSB != USB_BAT)
        exitUSB = NO_USB;
}

EXIT_USB_T USBapp_exitCDC (void)
{
    return exitUSB;
}

void USBapp_exitCDCState (EXIT_USB_T usbState)
{
    exitUSB = usbState;
}

void USBapp_CdcEmulatorInitialize(void)
{
    // Initialize CDC behavior
    CDCInitEP();
    line_coding.bCharFormat = 0;
    line_coding.bDataBits = 8;
    line_coding.bParityType = 0;
    line_coding.dwDTERate = 19200;
    EUSART_Initialize();
    // Initialize the arrays
    for (unsigned char i = 0; i <sizeof (USB_Buffer); i++)
    {
        USB_Buffer[i] = 0;
    }
    NextUSBOut = 0;
    LastRS232Out = 0;
}

void USBapp_DeviceCdcEmulatorHandler(void)
{
    // If USB is not prepared or in holding; exit function
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl == 1))
        return;
    // Capture data sent over USB in Serial format
    if (RS232_Out_Data_Rdy == 0)  // only check for new USB buffer if the old RS232 buffer is
    {						  // empty.  This will cause additional USB packets to be NAK'd
        LastRS232Out = getsUSBUSART(USB_Buffer,64); //until the buffer is free.
        if(LastRS232Out > 0)
        {
            RS232_Out_Data_Rdy = 1;  // signal buffer full
            RS232cp = 0;  // Reset the current position
        }
    }
    //Check if one or more bytes are waiting in the physical UART transmit
    //queue.  If so, send it out the UART TX pin.
    if(RS232_Out_Data_Rdy && mTxRdyUSART())
    {
        //Hardware flow control not being used.  Just send the data.
        EUSART_Write(USB_Buffer[RS232cp]);
        ++RS232cp;
        if (RS232cp == LastRS232Out)
            RS232_Out_Data_Rdy = 0;
    }
    //Check if we received a character over the physical UART, and we need
    //to buffer it up for eventual transmission to the USB host.
    if (EUSART_DataReady)       // See if Bytes are available
    {
        if(mDataRdyUSART() && (NextUSBOut < (CDC_DATA_OUT_EP_SIZE - 1)))
        {
            NOP();
            USB_Buffer[NextUSBOut] = EUSART_Read();
            ++NextUSBOut;
            USB_Buffer[NextUSBOut] = 0;
            rxByteFlag = 0;     // Clear for next captured byte
        }
    }
    //Check if any bytes are waiting in the queue to send to the USB host.
    //If any bytes are waiting, and the endpoint is available, prepare to
    //send the USB packet to the host.
    if((USBUSARTIsTxTrfReady()) && (NextUSBOut > 0))
    {
        putUSBUSART(&USB_Buffer[0], NextUSBOut);
        MOTEapp_setResponseReady();
        NextUSBOut = 0;
    }
    // Handle USB Transmit functionality
    CDCTxService();
}

void USBapp_LEDUpdateStatus(void)
{
    // LED Counter tick
    static uint16_t ledCount = 0;
    // Keep LED Off if USB is suspended
    if(USBIsDeviceSuspended() == true)
    {
        LED_GREEN_LAT = LED_OFF;
        return;
    }
    // Handle Blink Rate
    switch(USBGetDeviceState())
    {
        case CONFIGURED_STATE:
            /* We are configured.  Blink fast.
             * On for 75ms, off for 75ms, then reset/repeat. */
            if(ledCount == 1)
            {
                LED_GREEN_LAT = LED_ON;
            }
            else if(ledCount == 75)
            {
                LED_GREEN_LAT = LED_OFF;
            }
            else if(ledCount > 150)
            {
                ledCount = 0;
            }
            break;
        default:
            /* We aren't configured yet, but we aren't suspended so let's blink with
             * a slow pulse. On for 50ms, then off for 950ms, then reset/repeat. */
            if(ledCount == 1)
            {
                LED_GREEN_LAT = LED_ON;
            }
            else if(ledCount == 50)
            {
                LED_GREEN_LAT = LED_OFF;
            }
            else if(ledCount > 950)
            {
                ledCount = 0;
            }
        break;
    }
    /* Increment the millisecond counter. */
    ledCount++;
}

void USBapp_setrxByteFlag(void)
{
    rxByteFlag = 1;
}

uint8_t USBapp_getrxByteFlag(void)
{
    return rxByteFlag;
}

void USART_mySetLineCodingHandler(void)
{
    //Update the baudrate info in the CDC driver
    CDCSetBaudRate(cdc_notice.GetLineCoding.dwDTERate);
    //Update the baudrate of the UART
    uint32_t dwBaud;
    dwBaud = ((_XTAL_FREQ/4) / line_coding.dwDTERate) - 1;
    SPBRG = (uint8_t) dwBaud;
    SPBRGH = (uint8_t)((uint16_t) (dwBaud >> 8));
    // Init Driver
    EUSART_Initialize();
}

/***            USB EXTERNALLY REQUIRED FUNCTION                ***/
/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
bool USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, uint16_t size)
{
    switch( (int) event )
    {
        case EVENT_TRANSFER:
            break;

        case EVENT_SOF:
            /* We are using the SOF as a timer to time the LED indicator.  Call
             * the LED update function here. */
//            USBapp_LEDUpdateStatus();
            break;

        case EVENT_SUSPEND:
            /* Update the LED status for the suspend event. */
//            USBapp_LEDUpdateStatus();
            break;

        case EVENT_RESUME:
            /* Update the LED status for the resume event. */
//            USBapp_LEDUpdateStatus();
            break;

        case EVENT_CONFIGURED:
            /* When the device is configured, we can (re)initialize the
             * demo code. */
            USBapp_CdcEmulatorInitialize();
            break;

        case EVENT_SET_DESCRIPTOR:
            break;

        case EVENT_EP0_REQUEST:
            /* We have received a non-standard USB request.  The HID driver
             * needs to check to see if the request was for it. */
            USBCheckCDCRequest();
            break;

        case EVENT_BUS_ERROR:
            break;

        case EVENT_TRANSFER_TERMINATED:
            break;

        default:
            break;
    }
    return true;
}