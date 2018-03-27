/*********************************************************************
 *
 *      Microchip LoRa Mote Example Application
 *
 *********************************************************************
 * FileName:        MOTEapp.h
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

#ifndef MOTE_APP_H
#define MOTE_APP_H

#include <stdint.h>
#include "buttons.h"
#include "TMRapp.h"

typedef enum {MOTE_HARDWARE, MOTE_STARTUP, MOTE_JOINING, MOTE_RUNNING, MOTE_SWAP} MOTE_MODE_T;

void moduleResync(void);
void MOTEapp_handlerState(MOTE_MODE_T);
MOTE_MODE_T moteHandler(void);

void MOTEapp_setResponseReady(void);
bool MOTEapp_getModuleResponse(void);

bool MOTEapp_getResponseExpected(void);
void MOTEapp_responseBuffer(uint8_t);

void MOTEapp_TimerBasedTask(appTimerHandlerBit_t timerTask);

void moteApp_toggleLeds(bool);
void moteApp_clearCommand(void);
uint8_t MOTEapp_GetDataRateSetting(void);

void moteApp_delayms(uint16_t);
void moteApp_testSensors(void);

#endif
