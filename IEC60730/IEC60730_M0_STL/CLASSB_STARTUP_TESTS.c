/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     CLASSB_STARTUP_TEST.c                                                                             *
* @version  V1.00                                                                                             *
* $Date: 21/03/09 4:08p $                                                                                     *
* @brief    CLASSB Startup Time tests                                                                         *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include "stdio.h"
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"
#include "IEC60730_SAFETY_TEST_LIB.h"

extern uint8_t CLASSB_SAFE_STATE(uint8_t u8TestResult);	
extern uint32_t volatile g_u32RAMStartAddr;
extern uint32_t volatile g_u32RAMEndAddr;
/*---------------------------------------------------------------------------------------------------------*/
/*  Startup Tests                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t CLASSB_STARTUP_TESTS(void)
{

    uint8_t u8TestResult;
    /* CPU Registers Test */
    rt_printf("CPU Register Test (BIST)...");
    u8TestResult = CLASSB_Registers_Test(STARTUP);
    if(u8TestResult)
        rt_printf("Pass !!\n");
    else 
        CLASSB_SAFE_STATE(CPU_TEST_FAIL);
    
    /* Program Counter Test */
    rt_printf("Program Counter Test ...");
    u8TestResult = CLASSB_ProgramCounter_Test();
    if(u8TestResult)
        rt_printf("Pass !!\n");
    else 
        CLASSB_SAFE_STATE(PC_TEST_FAIL);

    return u8TestResult;
}
