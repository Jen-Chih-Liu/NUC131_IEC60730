/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     stack_test_runtime.c                                                                              *
* @version  V1.00                                                                                             *
* $Date: 21/03/10 1:21p $                                                                                     *
* @brief    IEC60730 Program Counter-Stack Test                                                               *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include <stdio.h>
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"
#include "IEC60730_SAFETY_TEST_LIB.h"

extern uint32_t g_u32STACK_TEST_PTRN[];
void IEC60730_Stack_Test_RunTime_Init(void)
{
    /* stack pattern is destoried by RAM test at startup stage */
    g_u32STACK_TEST_PTRN[0] = STACK_OVERRUN_PTRN0;
    g_u32STACK_TEST_PTRN[1] = STACK_OVERRUN_PTRN1;
    g_u32STACK_TEST_PTRN[2] = STACK_OVERRUN_PTRN2;
    g_u32STACK_TEST_PTRN[3] = STACK_OVERRUN_PTRN3;
}
uint8_t IEC60730_Stack_Test_RunTime() 
{
    uint32_t *pu32stackptrn;

    pu32stackptrn = g_u32STACK_TEST_PTRN;        // For SW test breakpoint
    if ((*pu32stackptrn++ == STACK_OVERRUN_PTRN0) &&
        (*pu32stackptrn++ == STACK_OVERRUN_PTRN1) &&
        (*pu32stackptrn++ == STACK_OVERRUN_PTRN2) &&
        (*pu32stackptrn   == STACK_OVERRUN_PTRN3) )

        return TEST_PASS;
    else
        return TEST_FAIL;
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
