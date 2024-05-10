/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     interrupt_test_runtime.c                                                                          *
* @version  V1.00                                                                                             *
* $Date: 21/03/09 4:08p $                                                                                     *
* @brief    IEC60730 Interrupt Test                                                                           *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include <stdio.h>
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"
#include "IEC60730_SAFETY_TEST_LIB.h"
extern uint32_t g_u32LSFreq;
extern uint32_t g_u32HSFreq;

void IEC60730_LSCLOCK_INIT(void)
{
    WriteGlobal((uint32_t *)&g_u32LSFreq, 0);
}
void IEC60730_HSCLOCK_INIT(void)
{
    WriteGlobal((uint32_t *)&g_u32HSFreq, 0);
}
void IEC60730_LSCLOCK_INT(void)
{
    uint32_t u32value;
    uint32_t u32result;

    u32result = ReadGlobal((uint32_t *)&g_u32LSFreq, &u32value);
    if (u32result == 0)
        WriteGlobal((uint32_t *)&g_u32LSFreq, 0);
    else
    {
        u32value++;
        WriteGlobal((uint32_t *)&g_u32LSFreq, u32value);
    }
}
void IEC60730_HSCLOCK_INT(void)
{
    uint32_t u32value;
    uint32_t u32result;

    u32result = ReadGlobal((uint32_t *)&g_u32HSFreq, &u32value);
    if (u32result == 0)
        WriteGlobal((uint32_t *)&g_u32HSFreq, 0);
    else
    {
        u32value++;
        WriteGlobal((uint32_t *)&g_u32HSFreq, u32value);
    }

    /* Overflow handling */
    if(u32value==0)
    {
        IEC60730_LSCLOCK_INIT();
        IEC60730_HSCLOCK_INIT();
    }
}

void IEC60730_Interrupt_Test_RunTime_Init(void)
{
    IEC60730_LSCLOCK_INIT();
    IEC60730_HSCLOCK_INIT();
}
uint8_t IEC60730_Interrupt_Test_RunTime()
{
    uint32_t u32Ratio, u32RatioH, u32RatioL, u32MeanError;
    uint32_t u32HSFreq;
    uint32_t u32LSFreq;
    uint32_t u32result;
    uint8_t u8TestResult;

    u32result = ReadGlobal((uint32_t *)&g_u32HSFreq, &u32HSFreq);
    if (u32result == 0)
        return TEST_FAIL;
    u32result = ReadGlobal((uint32_t *)&g_u32LSFreq, &u32LSFreq);
    if (u32result ==  0)
        return TEST_FAIL;

    if ((u32HSFreq < HSCLOCK_FREQ) || (u32LSFreq < LSCLOCK_FREQ))
        return TEST_PASS;

    u32Ratio = (uint32_t)(u32HSFreq/u32LSFreq);
    u32MeanError = (uint32_t)((HSCLOCK_FREQ/LSCLOCK_FREQ)*(uint8_t)CLOCK_DEVATION/100);
    u32RatioH = u32Ratio + u32MeanError;
    if(u32Ratio >= u32MeanError)
        u32RatioL = u32Ratio - u32MeanError;
    else
        u32RatioL = 0;
        
    if((u32Ratio > u32RatioH) || (u32Ratio < u32RatioL))         //For SW test breakpoint
        u8TestResult = TEST_FAIL;
    else
        u8TestResult = TEST_PASS;
        
    return u8TestResult;
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
