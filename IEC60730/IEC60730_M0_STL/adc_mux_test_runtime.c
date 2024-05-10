/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     adc_mux_test_runtime.c                                                                            *
* @version  V1.00                                                                                             *
* $Date: 21/03/09 4:08p $                                                                                     *
* @brief    IEC60730 ADC/MUX Test                                                                             *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include <stdio.h>
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"
#include "IEC60730_SAFETY_TEST_LIB.h"

static volatile uint32_t s_u32ConversionData;

uint8_t IEC60730_ADC_Test_RunTime(void) 
{
    /* ADC Test Implementations */
    if ((ADC->ADCR & ADC_ADCR_ADEN_Msk) == 0)
    {
        /* Power on ADC */
        ADC_POWER_ON(ADC);
        /* Configure ADC: single-end input, single scan mode, enable ADC analog circuit. */
        ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, VBG_CHANNELBIT);
			 ADC_CONFIG_CH7(ADC,ADC_ADCHER_PRESEL_INT_BANDGAP);
    }

    if (ADC_IS_DATA_VALID(ADC, VBG_CHANNEL) == 0)
    {
        /* Start A/D conversion */
        ADC_START_CONV(ADC);
        return TEST_PASS;
    }
    else
    {
        /* Get the conversion result */
        s_u32ConversionData = ADC->ADDR[VBG_CHANNEL] & ADC_ADDR_RSLT_Msk;
        /* Disable ADC module */
        //ADC_Close(ADC);
        /* Disable ADC */
        //ADC_POWER_DOWN(ADC);

        if ((s_u32ConversionData<BAND_GAP_H) && (s_u32ConversionData>BAND_GAP_L))        // For SW test breakpoint
            return TEST_PASS;
        else
            return TEST_FAIL;
    }
}
uint8_t IEC60730_MUX_Test_RunTime(void) 
{
    volatile uint32_t u32MuxData;

    /* MUX Test Implementations */
    if ((ADC->ADCR & ADC_ADCR_ADEN_Msk) == 0)
    {
        /* Power on ADC */
        ADC_POWER_ON(ADC);
        /* Configure ADC: single-end input, single scan mode, enable ADC analog circuit. */
        ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, MUX_CHANNELBIT);
    }

    if (ADC_IS_DATA_VALID(ADC, MUX_CHANNEL) == 0)
    {
        /* Start A/D conversion */
        ADC_START_CONV(ADC);
        return TEST_PASS;
    }
    else
    {
        /* Get the conversion result */
        u32MuxData = ADC->ADDR[MUX_CHANNEL] & ADC_ADDR_RSLT_Msk;
        /* Disable ADC module */
        ADC_Close(ADC);
        /* Disable ADC */
        ADC_POWER_DOWN(ADC);

        if (u32MuxData != s_u32ConversionData)        //For SW test breakpoint
            return TEST_PASS;
        else
            return TEST_FAIL;
    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
