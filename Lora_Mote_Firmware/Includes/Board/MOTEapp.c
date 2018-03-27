/*********************************************************************
 *
 *      Microchip LoRa Mote Example Application
 *
 *********************************************************************
 * FileName:        MOTEapp.c
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
#include "HardwareProfile.h"
#include "pin_manager.h"
#include "MOTEapp.h"
#include "USBapp.h"
#include "SSD1306oLED.h"
// Communication Includes
#include "eusart.h"
#include "buttons.h"
#include "TMRapp.h"
#include "tmr1.h"
#include "tmr2.h"
#include "input.h"
#include "adc.h"
#include "SSD1306oLED.h"
#include "memory.h"

typedef enum {moteStartup, moteReset, moteJoinMethod, moteCheckAbp, moteWaitAbp, moteCheckOtaa, moteWaitOtaa, moteJoined, moteGetDr, /*moteGetDr, moteSetAdr, moteSetDr, moteDoneDr, */moteNeedKeys, moteComFailure} MOTE_T;
MOTE_T  activeState = moteStartup;

static uint8_t moduleResonseBuffer[64];
static uint8_t responsePortNum[3];
static uint8_t responseId[16];
static uint8_t responseData[32];

static bool responseReady = false;
static MOTE_MODE_T moteState = MOTE_STARTUP;

typedef enum {runningEnter, runningSensor, runningSensorMenu, runningDataRateDisplay, runningDataRateSelect, runningSensorDisplay, runningUplink, runningUplinkSelect, runningUplinkTransmit, runningPortNumber, runningDownlink, runningDownlinkDisplay, runningMenuTimeout, runningPeriodicWakeUpLength, /* runningPeriodic, */runningPeriodicDisplay, runningSleep} MOTE_RUNNING_T;
MOTE_RUNNING_T  runningState = runningEnter;

static void EndLine(void);
static void sendCommand(const char *);
static void sendDataCommand(const char *string, uint8_t*, uint8_t);
static void sendSetCommand(uint8_t*);
static MOTE_T moteJoiningProcess(bool, bool, bool);
static MOTE_RUNNING_T moteRunningProcess(bool, bool, bool, bool);
uint8_t MOTEapp_getByteCounte(void);
static uint8_t unhex(uint8_t);
static uint8_t hexToStr(uint8_t);
static uint8_t moteApp_GoToSleep(void);
static void moteApp_clearBuffers(void);
void moteApp_add8bToDataBuffer(uint8_t, uint8_t);
void moteApp_add16bToDataBuffer(uint16_t, uint8_t);
uint8_t moteApp_addToDataBuffer(const char *, uint8_t);
uint16_t moteApp_convertSensorValue(const char *);

uint8_t* moteApp_getTempValue(void);
uint8_t* moteApp_getLightValue(void);
void moteApp_setSensorsInput(void);
void MOTEapp_clearModuleResponse (void) ;

void moteApp_setLightString(uint8_t*);
uint8_t moteApp_lightStringSize (void);
uint8_t* moteApp_getLightString(void);

void MOTEapp_SetDataRateSetting(uint8_t);

#define COM_RETRY_ATTEMPTS  10
static uint8_t appDevEui[16];
static uint8_t appKey[32];
static uint8_t retryCount = 0;
static uint8_t moduleBufferIndex = 0;

static uint16_t light = 0;
static uint16_t temperature = 0;
static uint8_t randomPortNum = 0;
static bool    messageReceived = false;

static uint8_t periodicUpValue = 3;
static uint8_t periodicTimerValue = 3;

static uint8_t dataRateValue = 2;

static bool menuTimeout = true;
typedef enum {periodicDisabled, periodicFive, periodicTen, periodicQuarter, periodicHalf, periodicFull} PERIODIC_T;
static PERIODIC_T periodicState = periodicQuarter;

#define SleepTimeOut        10          // In Secs; This is for Menu Timout
#define LightCapTime        100          // * 10 mSec; Captures the light and mV of PIC

static uint8_t secTicker = 0;
static uint8_t minTicker = 0;
static uint16_t m10SecTicker = 0;
static uint8_t sleepTicker = 0;

typedef enum {rn2483, rn2903} RN_T;
static RN_T rnModule = rn2483;

void moduleResync(void)
{
    bool escapeEvent = false;
    uint16_t escapeCounter = 0;
          
    // In some cases the module may be in sleep. This function makes sure the module 
    // is awake and synchronized at the right baudrate. This also works if the
    // module was already awake. Currently there's a bug in the module fw which
    // makes it misinterpret the firs command after resynchronizing.

    // First send wake up / restart autobaud
    TXSTAbits.TXEN = 0;
    EUSART_TX_LAT = 0;
    moteApp_delayms(10);
    TXSTAbits.TXEN = 1;
    moteApp_delayms(5);
    // Now send autobaud detect character
    TXREG = 0x55;
    while (TXSTAbits.TRMT == 0)
    {};
    // Now send a command to be ignored
    sendCommand("z");
    
    moteApp_delayms(40);
    while (escapeEvent)
    {
        if(MOTEapp_getModuleResponse())
           escapeEvent = true; 
        else if (++escapeCounter > 5000)
            escapeEvent = true; 
    }
}

void moteApp_delayms(uint16_t length)
{
    uint16_t counter;
    for(counter = 0; counter < length; counter++)
    {
        __delay_ms(1);
    }
}
void moduleSleep()
{
    sendCommand("sys sleep 999999999");     // Tell Module to Sleep till a Resync is heard
    while(1 == PIE1bits.TXIE)
        ;
    __delay_ms(16);
}

void MOTEapp_handlerState(MOTE_MODE_T state)
{
    moteState = state;
}

MOTE_MODE_T moteHandler(void)
{
    bool gotoSleep = false;
    bool gotoNextState = false;
    bool s2Button = false;
    bool readMessage = false;
    buttonStates_t buttonPress = Released;

    // Check USB connection state
    if (USB_DET_PORT)
    {
        if (USBapp_exitCDC() == NO_USB)
            moteState = MOTE_SWAP;
    } else {
        USBapp_exitCDCState(NO_USB);
    }

    // Get Timer Ticks
    if ( appTimer.mSecTick )
    {
        // Clear Flags for next task
        TMRapp_ClearBitFlags();
        // Execute Task
        TMRapp_Handler();
        MOTEapp_TimerBasedTask(appTimer);
    }

    // Handle button press events for menu navigation
    buttonPress = service_pushbutton();
    if (buttonPress == ShortPress1)
    {
        secTicker = 0;
        gotoNextState = true;   // Button Event has occured
    }
    else if (buttonPress == ShortPress2)
    {
        secTicker = 0;
        s2Button = true;   // Button Event has occured
    }

    // Gather response from LoRa Module if available
    if ( MOTEapp_getModuleResponse() )
        readMessage = true;
    else
        readMessage = false;

    // Handle Timed Task
    if (runningState == runningSensorMenu) 
    {
        // Get Sensor Data every 10mS
        if (m10SecTicker >= LightCapTime)
        {         
            NOP();
            oled_putString("    ", 6, 1);
            oled_putString(moteApp_getLightValue(), 6, 1);
            temperature = moteApp_convertSensorValue(moteApp_getTempValue());
            temperature = ADC_TempConversion(temperature); 
            oled_putint8(temperature, 6, 2);
            m10SecTicker = 0;
        }
    }
    // Sleep Event if idle
    if(secTicker)
    {
        if ((secTicker >= SleepTimeOut) && (menuTimeout))
        {
            gotoSleep = true;
            secTicker = 0;
        }
    }

    switch(moteState)
    {
        default:        // Invalid
            while(1);   // Hold For Error;
        break;
        case MOTE_STARTUP:
            oled_clear();
            oled_putString("MOTE Mode",1,0);
            oled_putString("Starting",2,1);
            oled_putString("ReRun PwrTst<S2>",0,3);
            activeState = moteStartup;
            moteState = MOTE_JOINING;
        break;
        case MOTE_JOINING:
            if ( moteJoiningProcess(readMessage, gotoNextState, s2Button) )
            {
                oled_clear();
                oled_putString("Mobile Mote -App",0,0);
                oled_putString("Navigate  Select", 0,2);
                oled_putString(" (S2)      (S3) ", 0,3);
                runningState = runningEnter;
                moteState = MOTE_RUNNING;
            }
        break;
        case MOTE_RUNNING:
            moteRunningProcess(gotoNextState, s2Button, readMessage, gotoSleep);
        break;
        case MOTE_SWAP:
            // Do Nothing
        break;
    }
    return moteState;
}

static MOTE_T moteJoiningProcess(bool messageReady, bool changeStates, bool selectButton)
{
    bool joiningComplete = false;
        
    switch (activeState)
    {
        default:        // Invalid
            while(1);   // Hold for Error
        break;
        case moteStartup:
            sendCommand("sys reset");
            activeState = moteReset;
        break;
        case moteReset:
            if (selectButton)
            {
                activeState = moteComFailure;
            }
            if ( messageReady )
            {
                if(strncmp(moduleResonseBuffer, "RN24", 4) == 0)   // Check 1st [2] Bytes for RN
                {
                    rnModule = rn2483;
                    oled_clear();
                    oled_putString(" Joining Method ",0,0);
                    oled_putString("<- ABP   OTAA ->", 0,2);
                    oled_putString("  (S2)   (S3)   ", 0,3);
                    activeState = moteJoinMethod;
                } 
                else if(strncmp(moduleResonseBuffer, "RN29", 4) == 0)   // Check 1st [2] Bytes for RN
                {
                    rnModule = rn2903;
                    oled_clear();
                    oled_putString(" Joining Method ",0,0);
                    oled_putString("<- ABP   OTAA ->", 0,2);
                    oled_putString("  (S2)   (S3)   ", 0,3);
                    activeState = moteJoinMethod;
                }
                else
                {
                    sendCommand("sys reset");   // Resend Previous Command
                    moteApp_delayms(25);
                    activeState = moteStartup;
                }
            }
        break;
        case moteJoinMethod:
            if (changeStates)
            {
                oled_putString("  (S2)          ", 0,3);
                moteApp_delayms(500);
                sendCommand("mac join abp");
                activeState = moteCheckAbp;
            }
            else if (selectButton)
            {
                oled_putString("         (S3)   ", 0,3);
                moteApp_delayms(500);
                sendCommand("mac join otaa");
                activeState = moteCheckOtaa;
            }
        break;
        case moteCheckAbp:
            if ( messageReady )
            {
                if(strcmp(moduleResonseBuffer, "ok") == 0) // Clearance is granted
                {
                    activeState = moteWaitAbp;
                }
                else if(strcmp(moduleResonseBuffer, "keys_not_init") == 0) // Bad keys give; Print Error Message
                {
                    oled_clear();
                    oled_putString("Valid Keys",0,0);
                    oled_putString("Required",0,1);
                    activeState = moteNeedKeys;   // Go back a state to handle module response
                }
                else // Retry Command
                {
                    sendCommand("mac join abp");    // Retry
                    activeState = moteCheckAbp;
                }
            }
        break;
        case moteWaitAbp:
            if ( messageReady )
            {
                if(strcmp(moduleResonseBuffer, "accepted") == 0) // Clearance is granted
                {
                    oled_clear();
                    oled_putString("ABP Join ",0,0);
                    oled_putString("Successful",0,1);
                    activeState = moteJoined;
                }
                else // Retry Command
                {
                    sendCommand("mac reset");   // Reset
                    activeState = moteReset;
                }
            }
        break;

        case moteCheckOtaa:
            if ( messageReady )
            {
                if(strcmp(moduleResonseBuffer, "ok") == 0) // Clearance is granted
                {
                    activeState = moteWaitOtaa;
                }
                else if(strcmp(moduleResonseBuffer, "keys_not_init") == 0) // Bad keys give; Print Error Message
                {
                    oled_clear();
                    oled_putString("Valid Keys",0,0);
                    oled_putString("Required",0,1);
                    activeState = moteNeedKeys;   // Go back a state to handle module response
                }
                else if(strcmp(moduleResonseBuffer, "invalid_param") == 0) // Clearance denied; Retry
                {
                    sendCommand("mac join otaa");    // Retry
                    activeState = moteCheckOtaa;
                }
                else if(strcmp(moduleResonseBuffer, "no_free_ch") == 0) // Clearance denied; Retry
                {   // Restore to Startup state; This will reset the RN Module allowing for channels to free up
                    activeState = moteStartup;
                }
            }
        break;
        case moteWaitOtaa:
            if ( messageReady )
            {
                if(strcmp(moduleResonseBuffer, "accepted") == 0) // Clearance is granted
                {
                    oled_clear();
                    oled_putString("OTAA Join ",0,0);
                    oled_putString("Successful",0,1);
                    activeState = moteJoined;
                }
                else if(strcmp(moduleResonseBuffer, "denied") == 0) // Bad keys give; Print Error Message
                {
                    oled_clear();
                    oled_putString("OTAA Join",1,0);
                    oled_putString("Denied",2,1);
                    sendCommand("mac join otaa");
                    activeState = moteCheckOtaa;   // Go back a state to handle module response
                }
                else if(strcmp(moduleResonseBuffer, "no_free_ch") == 0) // Clearance denied; Retry
                {
                    activeState = moteStartup;
                }
            }
        break;
        case moteJoined:
            sendCommand("mac get dr");
            activeState = moteGetDr;
        break;
        case moteGetDr:
            if ( messageReady )
            {
                if(strcmp(moduleResonseBuffer, "invalid_param") == 0) // Clearance denied; Retry
                {
                    sendCommand("mac get dr");
                    activeState = moteGetDr;
                }
                else
                {
                    uint8_t readDataRate;
                    readDataRate = moduleResonseBuffer[0] - '0';
                    dataRateValue = readDataRate;
                    joiningComplete = true;     // Action Done; Move on to Running
                }
            }
        break;
        case moteNeedKeys:
            if (selectButton)
            {
                activeState = moteComFailure;
            }
            // Do Nothing; Wait for USB; Hold for Display
        break;
        case moteComFailure:
            DATAEE_WriteByte(0x00, 0x00);
            while((!SWITCH_1_PORT) && (!SWITCH_2_PORT));
            __delay_ms(16);
            __delay_ms(16);
            __delay_ms(16);
            RESET();
            // Do Nothing; Wait for USB; Hold for Display
        break;
    }
    return joiningComplete;
}

static uint8_t dotCount = 0;
static uint8_t dataBuffer[16];
static uint8_t dataRatePosition = 0;
static uint8_t dataRateSelection = 2;

static MOTE_RUNNING_T moteRunningProcess(bool changeStates, bool selectButton, bool messageReady, bool sleepEvent)
{
    uint8_t readByte = 0;
    
    if ((sleepEvent) && (periodicState))
    {
        if (sleepTicker >= periodicTimerValue) // No Manual Transmission for a time; Do one Auto
        {   
            // Prepare Buffers and data
            moteApp_clearBuffers();
            // Make Sure Port is in allowed Range
            // Prepare DataBuffer for Tx
            light = 0;
            temperature = 0;
            NOP();           
            // Measure Sensors
            moteApp_setSensorsInput();
            uint8_t sizeOfUpdate = 0;
            light = moteApp_convertSensorValue(moteApp_getLightValue());
            oled_putString(moteApp_getLightString(), 6, 1);
            sizeOfUpdate = moteApp_addToDataBuffer(moteApp_getLightString(), 4);
            
            temperature = moteApp_convertSensorValue(moteApp_getTempValue());
            temperature = ADC_TempConversion(temperature); 
            moteApp_add8bToDataBuffer(temperature, 4 + moteApp_lightStringSize() + 1);

            // Do Normal Operation
            sleepTicker = 0;
            oled_clear();
            oled_putString("Send Periodic Tx",0,0);
            oled_putString("On Port #: ",0,1);
            oled_putString("L:       T: ",0,2);
            oled_putString(moteApp_getLightString(), 3, 2);
            oled_putint8(temperature,11,2);
            // Getting Random Channel
            randomPortNum = TMR2_ReadTimer();
            // Make Sure Port is in allowed Range
            if (!randomPortNum)
                randomPortNum = 5;
            else if (randomPortNum > 223)
                randomPortNum = randomPortNum - 50;
            //Print Port
            oled_putUint8(randomPortNum,10,1);
            // Prepare DataBuffer for Tx
            moteApp_add8bToDataBuffer(randomPortNum, 0);
            dataBuffer[3] = 0x20;
            NOP();
            sendDataCommand("mac tx uncnf ", dataBuffer, 12);
            secTicker = SleepTimeOut - (SleepTimeOut / 3);
            runningState = runningUplinkTransmit;
        }
        else
        {
                if (menuTimeout)
                {
                    sleepTicker++;
                    runningState = runningSleep;
                    selectButton = true;
                } else {
                    sleepTicker = 0;
                }
        }
    }

    // Menu Operation
    switch(runningState)
    {
        default:        // Invalid
            while(1);   // Hold for Error
        break;
        case runningEnter:
            if ( (changeStates) || (selectButton) )
            {
                oled_clear();
                oled_putString("  Issue Uplink  ",0,0);
                oled_putString(" LoRaWAN Packet ",0,1);
                oled_putString("(NAV)    (SEL) ",0,3);
                runningState = runningUplink;
            }
        break;
        case runningUplink:
            if (changeStates)
            {
                oled_clear();
                oled_putString(" View Downlink  ",0,0);
                oled_putString("  Last Message  ",0,1);
                oled_putString("(NAV)    (SEL) ",0,3);
                runningState = runningDownlink;
            }
            else if (selectButton)
            {
                sleepTicker = 0;
                // Prepare Display
                oled_clear();
                oled_putString("Uplink Pckt Type",0,0);
                oled_putString("Throu Port# ",0,1);
                
                oled_putString("(UNCNF)    (CNF) ",0,3);
                // Prepare Buffers and data
                moteApp_clearBuffers();
                randomPortNum = TMR2_ReadTimer();
                // Make Sure Port is in allowed Range
                if (!randomPortNum)
                    randomPortNum = 5;
                else if (randomPortNum > 223)
                    randomPortNum = randomPortNum - 50;
                oled_putUint8(randomPortNum,12,1);
                moteApp_add8bToDataBuffer(randomPortNum, 0);
                dataBuffer[3] = 0x20;
                // Prepare DataBuffer for Tx
                light = 0;
                temperature = 0;
                NOP();
                moteApp_setSensorsInput();
                moteApp_delayms(500);
                uint8_t sizeOfUpdate = 0;
                light = moteApp_convertSensorValue(moteApp_getLightValue());
                sizeOfUpdate = moteApp_addToDataBuffer(moteApp_getLightValue(), 4);
                NOP();
                NOP();
                temperature = moteApp_convertSensorValue(moteApp_getTempValue());
                temperature = ADC_TempConversion(temperature); 
                moteApp_add8bToDataBuffer(temperature, 4 + moteApp_lightStringSize() + 1);
                NOP();
                // Change State
                runningState = runningUplinkSelect;
            }
        break;
        case runningUplinkSelect:
            if (changeStates)
            {
                sendDataCommand("mac tx uncnf ", dataBuffer, 12);
                
            }
            if (selectButton)
            {
                sendDataCommand("mac tx cnf ", dataBuffer, 12);
            }

            if ( (changeStates) || (selectButton) )
            {
                oled_clear();
                oled_putString("Sent Data: ",0,0);
                oled_putString("Tx Port #: ",0,1);
                oled_putUint8(randomPortNum,12,1);
                oled_putString("L:       T: ",0,2);
                oled_putString(moteApp_getLightString(), 3, 2);
                oled_putint8(temperature,11,2);
                LED_GREEN_LAT = LED_OFF;
                runningState = runningUplinkTransmit;
            }
        break;
        case runningUplinkTransmit:
            if ( messageReady )
            {
                if(strcmp(moduleResonseBuffer, "ok") == 0)   // Check 1st [2] Bytes for RN
                {
                    LED_GREEN_LAT = LED_ON;
                    runningState = runningUplinkTransmit;
                }
                if(strncmp(moduleResonseBuffer, "mac_rx", 6) == 0)   // Check 1st [2] Bytes for RN
                {
                    uint8_t bufferReadPos = 7;
                    // Parse Buffer
                    
                    // Port ID
                    responsePortNum[0] = moduleResonseBuffer[bufferReadPos++];
                    responsePortNum[1] = moduleResonseBuffer[bufferReadPos++];
                    if (responsePortNum[1] != ' ')
                    {
                       responsePortNum[2] = moduleResonseBuffer[bufferReadPos++]; 
                       if (responsePortNum[2] != ' ')
                        {
                           bufferReadPos++;
                        }
                    }
                    // Capture Data for Display
                    for (uint8_t counter = 0; counter < sizeof(moduleResonseBuffer); counter++)
                    {
                        responseData[counter] = moduleResonseBuffer[bufferReadPos];
                        
                        if (responseData[counter] != 0x00)
                        {
                            bufferReadPos++;    //Increment
                        } else {
                            counter = sizeof(moduleResonseBuffer);
                        } 
                    }        
                    // Toggle Red to indicate available Rx; Clear Green to show Tx sent
                    LED_RED_LAT = 1;
                    LED_GREEN_LAT = LED_OFF;
                    // Indicate on oLED
                    oled_putString("Server Response ",0,2);
                    oled_putString("    Received    ",0,3);
                }
                else if(strncmp(moduleResonseBuffer, "mac_tx", 6) == 0)   // Check 1st [2] Bytes for RN
                {
                    LED_GREEN_LAT = LED_OFF;
                    oled_putString("Data Transmitted",0,3);
                }
                else if(strncmp(moduleResonseBuffer, "mac_er", 6) == 0)   // Check 1st [2] Bytes for RN
                {
                    oled_putString("Mac Error Occurred",0,3);
                }
                else if(strncmp(moduleResonseBuffer, "no_fre", 6) == 0)   // Check 1st [2] Bytes for RN
                {
                    oled_putString("No Free Channel",0,3);
                }
                __delay_ms(16);
            }
            if ( (changeStates) || (selectButton) )
            {
                oled_clear();
                oled_putString("  Issue Uplink  ",0,0);
                oled_putString(" LoRaWAN Packet ",0,1);
                oled_putString("(NAV)    (SEL) ",0,3);
                runningState = runningUplink;
            }
        break;
        case runningDownlink:
            if (changeStates)
            {
                oled_clear();
                oled_putString("  Sensor Data  ",0,0);
                oled_putString("    Display    ",0,1);
                oled_putString("(NAV)    (SEL) ",0,3);
                runningState = runningSensor;
            }
            else if (selectButton)
            {
                // Add comment about Rx Buffer sizing
                oled_clear();
                // Clear LED Indicator
                LED_RED_LAT = 0;
                // Display
                oled_putString("Downlink Display",0,0);
                // Line 1
                oled_putString("Rx Port# ",0,1);
                oled_putString(&responsePortNum,9,1);
                // Line 2
                oled_putString("Data: ",0,2);
                oled_putString(&responseData[0],6,2);
                // Line 3
                oled_putString(&responseData[10],0,3);
                // Change State
                runningState = runningDownlinkDisplay;
            }
        break;
        case runningDownlinkDisplay:
            if ( (changeStates) || (selectButton) )
            {
                oled_clear();
                oled_putString(" View Downlink  ",0,0);
                oled_putString("  Last Message  ",0,1);
                oled_putString("(NAV)    (SEL) ",0,3);
                runningState = runningDownlink;
            }
        break;
        case runningSensor:
            if (changeStates)
            {
                oled_clear();
                oled_putString("Menu Has Timeout",0,0);
                if (menuTimeout)
                    oled_putString("    Enabled   ", 0,2);
                else
                    oled_putString("    Disabled  ", 0,2);
                oled_putString("(NAV)    (CNG) ",0,3);
                runningState = runningMenuTimeout;
            }
            else if (selectButton)
            {
                oled_clear();
                oled_putString("  Sensor Data  ",0,0);
                oled_putString("Light:",0,1);
                oled_putString("Temp :",0,2);
                oled_putString("C",10,2);
                
                moteApp_setSensorsInput();
                runningState = runningSensorMenu;
            }
        break;
        case runningMenuTimeout:
            if (changeStates)
            {
                oled_clear();
                oled_putString("Periodic Uplink ",0,0);
                oled_putString("Tx Event Timing ",0,1);
                oled_putString("Every:",0,2);
                oled_putString("Mins",12,2);
                // Toggle Display
                switch (periodicUpValue)
                {
                    default:
                    case 0:
                        oled_putString("5",7,2);
                    break;
                    case 1:
                        oled_putString("10",7,2);
                    break;
                    case 2:
                        oled_putString("15",7,2);
                    break;
                    case 3:
                        oled_putString("30",7,2);
                    break;
                    case 4:
                        oled_putString("60",7,2);
                    break;
                    case 5:
                        oled_putString("OFF",7,2);
                    break;
                }
                oled_putString("(NAV)    (CNG) ",0,3);
                runningState = runningPeriodicWakeUpLength;
            }
            else if (selectButton)
            {
                if (menuTimeout)
                {
                    menuTimeout = false;
                    oled_putString("    Disabled  ", 0,2);
                }
                else
                {
                    menuTimeout = true;
                    oled_putString("    Enabled   ", 0,2);
                }     
            }
        break;
        case runningSensorMenu:
            //Display is handled at top of function
            if ( (changeStates) || (selectButton) )    // Wait for First Measurement
            {
                oled_clear();
                oled_putString("  Sensor Data  ",0,0);
                oled_putString("    Display    ",0,1);
                oled_putString("(NAV)    (SEL) ",0,3);
                runningState = runningSensor;
            }
        break;
        case runningPeriodicWakeUpLength:
            if (changeStates)
            {
                oled_clear();
                oled_putString("Select Data Rate",0,0);
                oled_putString("Data Rate: ",0,2);
                oled_putString("(NAV)    (CNG) ",0,3);
                runningState = runningDataRateDisplay;
                
                // Toggle Display
                switch (dataRateValue)
                {
                    default:
                    case 0:
                        oled_putString(" 0 ",10,2);
                    break;
                    case 1:
                        oled_putString(" 1 ",10,2);
                    break;
                    case 2:
                        oled_putString(" 2 ",10,2);
                    break;
                    case 3:
                        oled_putString(" 3 ",10,2);
                    break;
                    case 4:
                        oled_putString(" 4 ",10,2);
                    break;
                    case 5:
                        oled_putString(" 5 ",10,2);
                    break;
                    case 6:
                        oled_putString("ADR",10,2);
                    break;
                }
            }
            if (selectButton)
            {
                if (periodicUpValue < 5)
                {
                    periodicUpValue++;
                } else {
                    periodicUpValue = 0;
                }
                // Toggle Display
                switch (periodicUpValue)
                {
                    default:
                    case 0:
                        periodicTimerValue = 9;
                        oled_putString(" 5 ",7,2);
                    break;
                    case 1:
                        periodicTimerValue = 18;
                        oled_putString(" 10",7,2);
                    break;
                    case 2:
                        periodicTimerValue = 27;
                        oled_putString(" 15",7,2);
                    break;
                    case 3:
                        periodicTimerValue = 54;
                        oled_putString(" 30",7,2);
                    break;
                    case 4:
                        periodicTimerValue = 108;
                        oled_putString(" 60",7,2);
                    break;
                    case 5:
                        periodicTimerValue = 255;
                        oled_putString("OFF",7,2);
                    break;
                }
            }
        break;
        case runningDataRateDisplay:
            if (changeStates)
            {
                // Issue Commands
                MOTEapp_SetDataRateSetting(dataRateValue);
                // Change State
                oled_clear();
                oled_putString("Enter Sleep Mode",0,0);
                oled_putString("(NAV)    (SLP) ",0,3);
                runningState = runningSleep;
            }
            if (selectButton)
            {
                if (dataRateValue < 6)
                {                  
                    if ((rnModule == rn2903) && (dataRateValue > 3))
                        dataRateValue = 6;
                    else
                        dataRateValue++;
                } 
                else {
                    dataRateValue = 0;
                }
                // Toggle Display
                switch (dataRateValue)
                {
                    default:
                    case 0:
                        oled_putString(" 0 ",10,2);
                    break;
                    case 1:
                        oled_putString(" 1 ",10,2);
                    break;
                    case 2:
                        oled_putString(" 2 ",10,2);
                    break;
                    case 3:
                        oled_putString(" 3 ",10,2);
                    break;
                    case 4:
                        oled_putString(" 4 ",10,2);
                    break;
                    case 5:
                        oled_putString(" 5 ",10,2);
                    break;
                    case 6:
                        oled_putString("ADR",10,2);
                    break;
                }
            }
        break;
        case runningSleep:
            if (selectButton)
            {
                if( moteApp_GoToSleep() ) 
                {
                    changeStates = true;
                }
                else
                {
                    secTicker = SleepTimeOut - 1;
                }
            }
            if (changeStates)
            {
                oled_clear();
                oled_putString("  Issue Uplink  ",0,0);
                oled_putString(" LoRaWAN Packet ",0,1);
                oled_putString("(NAV)    (SEL) ",0,3);
                runningState = runningUplink;
            }
        break;
    }
    return runningState; // no return value needed but we declare one for this function.
}

void MOTEapp_SetDataRateSetting(uint8_t dr)
{
    if (dr != 8)
    {
        sendCommand("mac set adr off");
        __delay_ms(16);
        __delay_ms(16);
        __delay_ms(16);
    }

    switch (dr)
    {
        case 0:
            sendCommand("mac set dr 0");
        break;    
        case 1:
            sendCommand("mac set dr 1");
        break;
        case 2:
            sendCommand("mac set dr 2");
        break;
        case 3:
            sendCommand("mac set dr 3");
        break;
        case 4:
            sendCommand("mac set dr 4");
        break;
        case 5:
            sendCommand("mac set dr 5");
        break;
        case 6:
            sendCommand("mac set dr 6");
        break;
        case 7:
            sendCommand("mac set dr 7");
        break;
        case 8:
            sendCommand("mac set adr on");
        break;
    }
    __delay_ms(16);
    __delay_ms(16);
    __delay_ms(16);
    sendCommand("mac save");
}

void MOTEapp_TimerBasedTask(appTimerHandlerBit_t timerTask)
{
    // Excute 1mS task Regardless
    // --> 1mSec Task
    // Variable TimeLength Task
    if (timerTask.mSecTenTick)
    {
        // --> 10mSec Task
        m10SecTicker++;
    }
    if (timerTask.mSecHundredTick)
    {
        // --> 100mSec Task
        
    }
    if (timerTask.SecTick)
    {
        // --> 1Sec Task
        secTicker++;
    }
    if (timerTask.MinTick)
    {
        // --> 1Min Task
        minTicker++;
    }
    if (timerTask.HourTick)
    {
        // --> 1Hour Task
    }
    if (timerTask.HalfDayTick)
    {
        // --> 12Hour Task
    }
    if (timerTask.DayTick)
    {
        // --> 1Day Task
    }
}

void MOTEapp_clearModuleResponse (void) 
{
    
    for (uint8_t counter = 0; counter < 64; counter++)
    {
        moduleResonseBuffer[counter] = 0;
    }
}
bool MOTEapp_getModuleResponse(void)
{
    bool terminatorReceived = false;    // Return false; Unless Terminator character (0x0D) is captured
    
    if (EUSART_DataReady)       // See if Bytes are available
    {
        uint8_t bufferByte = 0; // Restore for read
        bufferByte = EUSART_Read();
        // See what we got
        if (bufferByte == 0x0D)
        {
            moduleResonseBuffer[moduleBufferIndex] = 0x00; // Add Null terminator
            terminatorReceived = true;
            moduleBufferIndex = 0; // Prepare index for next message
        }
        else if ( (bufferByte == 0x00) || (bufferByte == 0x0A) )
        {
            // Do nothing
        }
        else
        {
            moduleResonseBuffer[moduleBufferIndex] = bufferByte; // Add Byte to Buffer
            // Increment Buffer for next byte
            moduleBufferIndex++;
        }
    }
    return terminatorReceived;
}

void MOTEapp_setResponseReady(void)
{
    responseReady = true;   // Set Local Variable
}

static void EndLine(void)
{
    EUSART_Write(0x0D); // CR; Carriage Return
    EUSART_Write(0x0A); // NF; Line Feed
}

static void sendCommand(const char *string)
{
    // Prepare Buffer for Response
    while (*string)
    {
        EUSART_Write(*string++);
    }
    EndLine();   
}

static void sendDataCommand(const char *string, uint8_t* appData, uint8_t dataLength)
{
    // String
    while (*string)
    {
        EUSART_Write(*string++);
    }
    // Port No & space
    for (uint8_t byteCount = 0; byteCount < 3; byteCount++)
    {
        EUSART_Write(appData[byteCount]);
    }
    //Space
    EUSART_Write(appData[3]);
    //Data
    for (uint8_t byteCount = 4; byteCount < dataLength; byteCount++)
    {
        EUSART_Write(hexToStr(appData[byteCount] >> 4));
        EUSART_Write(hexToStr(appData[byteCount] & 0x0F));
    }
    EndLine();
}

static uint8_t unhex(uint8_t c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return '-';
}

static uint8_t hexToStr(uint8_t character)
{
    // Mask out Top Nibble
    character = character & 0x0F;
    // Check if: 0-9; Else if: A-F
    if (character < 0x0A)
    {
        return character + '0';
    }
    else if ( (character >= 0x0A) && (character <= 0x0F) )
    {
        return character + '0' + 7;
    }
    else
    {
        return 0;
    }
}

static uint8_t moteApp_GoToSleep(void)
{
    uint8_t status = 0;
    while (SWITCH_2_PORT == 0);     // Hold for Button Release
    moduleSleep();
    oled_displayOff(true); 
    ADCON0bits.ADON = 0;       // ADC Off
    INTCONbits.GIE = 0;
    GIE = 0;
    PEIE = 0;
    // LCD Backlight
    LCD_BACKLIGHT_LAT = DISABLE;
    PIE2bits.TMR3IE = 0;
    // Prepare USB Detect
    USB_DET_ANSEL = 1;
    // Prepare IOC to wake us up
    USB_DET_EN = 1;      // Enables CMP1 interrupt.
    SWITCH_1_IOC = 1;
    SWITCH_2_IOC = 1;
    MODULE_WAKE_IOC = 1;
    IOC_ENABLE = 1;
    // Dummy Reads to Establish IOC level
    status = SWITCH_1_PORT;
    status = SWITCH_2_PORT;
    status = MODULE_WAKE_PORT;
    
    if (periodicState)
    {
        WDTCONbits.SWDTEN = 1;
    }
    
    IOC_FLAG = 0;
    INT0_FLAG = 0;
    INT1_FLAG = 0;
    USB_DET_FLAG = 0;
    NOP();
    SLEEP();
    NOP();
    // Reconfigure Critical peripherals
    WDTCONbits.SWDTEN = 0;
    EUSART_FlushBuffer();
    // Disable IOC
    IOC_ENABLE = 0;
    SWITCH_1_IOC = 0;
    SWITCH_2_IOC = 0;
    MODULE_WAKE_IOC = 0;
    PIE2bits.C1IE = 0;
    GIE = 1;

    PIN_MANAGER_Initialize();
    // Process Wake Up Event
    if ((IOC_FLAG)||(INT0_FLAG)||(INT1_FLAG)||(USB_DET_FLAG))
    {   // PushButton or Module IOC woke us
        IOC_FLAG = 0;
        INT0_FLAG = 0;
        INT1_FLAG = 0;
        status = 1;
        
        // Prepare LCD
        oled_displayOff(false);
        oled_init();         // Display
        // Prepare LCD Backlight
        LCD_BACKLIGHT_LAT = ENABLE;
        TMR3H = 0xFB;       // Values Used to restore; Taken from TMR3.c
        TMR3L = 0x50;       // Values Used to restore; Taken from TMR3.c
        PIE2bits.TMR3IE = 1;
    }
    else
    {   // Time woke us. Reload; so wake from sleep is short
        secTicker = 10;
        status = 0;
        // Prepare LCD Backlight for quick sleep
        LCD_BACKLIGHT_LAT = DISABLE;
        PIE2bits.TMR3IE = 0;
    }
    USB_DET_ANSEL = 0;

    moduleResync();
    moteApp_delayms(40);
    moduleResync();
    moteApp_delayms(40);
    return status;
}

void moteApp_clearCommand(void)
{
    sendCommand("\r\n");
}

void moteApp_toggleLeds(bool state)
{
    __delay_ms(16);
        __delay_ms(16);
        __delay_ms(16);
    if (state)
    {
        sendCommand("sys set pindig GPIO10 1\r\n");
        __delay_ms(16);
        __delay_ms(16);
        __delay_ms(16);
        sendCommand("sys set pindig GPIO11 1\r\n");
    }
    else
    {
        sendCommand("sys set pindig GPIO10 0\r\n");
        __delay_ms(16);
        __delay_ms(16);
        __delay_ms(16);
        sendCommand("sys set pindig GPIO11 0\r\n");
    }
}

static void moteApp_clearBuffers(void)
{
    for (uint8_t counter = 0; counter < 3; counter++)
    {
        responsePortNum[counter] = 0;
    }
    for (uint8_t counter = 0; counter < 34; counter++)
    {
        responseId[counter] = 0;
    }
    for (uint8_t counter = 0; counter < 32; counter++)
    {
        responseData[counter] = 0;
    }
    for (uint8_t counter = 0; counter < sizeof(dataBuffer); counter++)
    {
        dataBuffer[counter] = 0;
    }
}

void moteApp_add8bToDataBuffer(uint8_t number, uint8_t bufferLocation)
{
    dataBuffer[bufferLocation + 2] = '0' + (number % 10);
    number /= 10;
    dataBuffer[bufferLocation + 1] = '0' + (number % 10);
    number /= 10;
    dataBuffer[bufferLocation] = '0' + (number % 10);
}

void moteApp_add16bToDataBuffer(uint16_t number, uint8_t bufferLocation)
{
    dataBuffer[bufferLocation + 4] = '0' + (number % 10);
    number /= 10;
    dataBuffer[bufferLocation + 3] = '0' + (number % 10);
    number /= 10;
    dataBuffer[bufferLocation + 2] = '0' + (number % 10);
    number /= 10;
    dataBuffer[bufferLocation + 1] = '0' + (number % 10);
    number /= 10;
    dataBuffer[bufferLocation] = '0' + (number % 10);
}

uint8_t moteApp_addToDataBuffer(const char *string, uint8_t bufferLocation)
{
    uint8_t index;
    for(index = 0; index < 6; index++)
    {
        if (*string != 0)
        {
            dataBuffer[bufferLocation++] = *string++;
        }
        else
        {
            dataBuffer[bufferLocation] = 0x20;  // Add Space Character to Tail
            break;
        }
    }
    return index;
}

uint16_t moteApp_convertSensorValue(const char *string)
{
    uint8_t index;
    uint16_t convertedValue = 0;
    for(index = 0; index < 4; index++)
    {
        if (*string != 0)
        {
            if (index != 0)                             // If not first byte shift for next number
                convertedValue = convertedValue * 10;   

            convertedValue = (*string++ - '0') + convertedValue; 
        }
        else
        {
            break;
        }
    }
    return convertedValue;
}

void moteApp_testSensors(void)
{
    // Configure GPIO pins
    moteApp_setSensorsInput();
    
    uint16_t dummyValue;
    oled_clear();
    oled_putString("ADC Test",0,0);
    oled_putString("Light =",0,1);
    oled_putString("Temp  =",0,2);
    oled_putChar('C',14,2);
    temperature = moteApp_convertSensorValue(moteApp_getTempValue());
    temperature = ADC_TempConversion(temperature); 
    oled_putint8(temperature, 10, 2);
    NOP();
    oled_putString("    ", 10, 1);
    oled_putString(moteApp_getLightValue(), 10, 1);
}

void moteApp_setSensorsInput(void)
{
    uint16_t escapeCounter = 0;
    sendCommand("sys set pinmode GPIO13 ana");
    while ( !MOTEapp_getModuleResponse() )
    {   // Wait for Response     
        if (++escapeCounter > 50000)
            break;
    }    
    escapeCounter = 0;
    sendCommand("sys set pinmode GPIO12 ana");
    while ( !MOTEapp_getModuleResponse() )
    {   // Wait for Response     
        if (++escapeCounter > 50000)
            break;
    }  
    moteApp_delayms(100);
}
uint8_t* moteApp_getTempValue(void)
{  
    uint16_t escapeCounter = 0;
    
    sendCommand("sys get pinana GPIO12");
    while ( !MOTEapp_getModuleResponse() )
    {   // Wait for Response     
        if (++escapeCounter > 50000)
            break;
    }
    
    return &moduleResonseBuffer;
}

uint8_t* moteApp_getLightValue(void)
{  
    uint16_t escapeCounter = 0;
    
    sendCommand("sys get pinana GPIO13");
    while ( !MOTEapp_getModuleResponse() )
    {   // Wait for Response     
        if (++escapeCounter > 50000)
            break;
    }    
    moteApp_setLightString(moduleResonseBuffer);
    return &moduleResonseBuffer;
}

uint8_t storedLight[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t lightStringSize;
uint8_t clearLight[5] = {0x00, 0x00, 0x00, 0x00, 0x00};       

void moteApp_setLightString(uint8_t* lightString)
{
    memcpy(storedLight, clearLight, sizeof(storedLight));   
    memcpy(storedLight, lightString, sizeof(storedLight));
    // Remove Null's
    for (uint8_t stingPos = 0; stingPos < sizeof(storedLight); stingPos++)
    {
        if (storedLight[stingPos] == 0x00)  // Null as terminator
        {
            lightStringSize = stingPos;
        }
    }
    NOP();
}

uint8_t moteApp_lightStringSize (void)
{
    return lightStringSize;
}
uint8_t* moteApp_getLightString(void)
{   
    return &storedLight;
}