// input.c

/*********************************************************************
* Software License Agreement:
*
* The software supplied herewith by Microchip Technology Incorporated
* (the "Company") for its PICmicro® Microcontroller is intended and
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
*********************************************************************/
#include <xc.h>
#include <stdint.h>
#include "adcSensors.h"
#include "adc.h"

#define LIGHT_CHANNEL   0x03
#define VOLT_CHANNEL    0x00

uint8_t adcS_getLight(void)
{
    static uint16_t  integrator = 0;
    uint16_t         averageValue;
    uint16_t          lightValue;
    
    integrator += ADC_GetConversion(LIGHT_CHANNEL);
    averageValue = integrator >> 5;
    integrator -= averageValue;
    lightValue = averageValue >> 2;
    return (uint8_t)lightValue;
}

uint16_t adcS_getMv (void)
{
    static uint16_t  V_PIC_ADC;
    uint16_t  averageValue;

    V_PIC_ADC +=  ADC_GetConversion(VOLT_CHANNEL); // Sample the ADC Ref
    averageValue = V_PIC_ADC >> 3;
    V_PIC_ADC -= averageValue;

    return (uint16_t)averageValue;
}