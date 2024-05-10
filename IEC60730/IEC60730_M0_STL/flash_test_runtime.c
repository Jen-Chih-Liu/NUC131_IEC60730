/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     flash_test_runtime.c                                                                              *
* @version  V1.00                                                                                             *
* $Date: 21/02/24 3:27p $                                                                                    *
* @brief    IEC60730 Intrnal Flash Test runtime                                                               *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"
#include "IEC60730_SAFETY_TEST_LIB.h"
static uint32_t s_u32RomCurrentAddr, s_u32RomEndAddr;
static uint32_t volatile s_u32CHECKSUM = 0;
extern uint32_t checksum_flash(uint8_t *string, uint32_t u32seed, uint32_t u32Length);

void IEC60730_Flash_Test_RunTime_Init(void)
{
    s_u32RomCurrentAddr = (uint32_t)ROM_START;
    s_u32RomEndAddr = s_u32RomCurrentAddr + ROM_LENGTH;
    s_u32CHECKSUM = CHECKSUM_SEED;
}

uint8_t IEC60730_Flash_Test_RunTime(void)
{
    uint32_t u32StartAddr;
    uint8_t u8TestResult;

    u32StartAddr = s_u32RomCurrentAddr;

    s_u32CHECKSUM = checksum_flash((uint8_t*)u32StartAddr, s_u32CHECKSUM, (uint32_t)ROM_RUNTIME_TEST_LENGTH);

    /* check if end address */		
    u32StartAddr += (uint32_t)ROM_RUNTIME_TEST_LENGTH;
    if (u32StartAddr >= s_u32RomEndAddr) {
        s_u32RomCurrentAddr = (uint32_t)ROM_START;
        if(s_u32CHECKSUM != __Check_Sum)        //For SW test breakpoint
            u8TestResult = TEST_FAIL;
        else
            u8TestResult = TEST_PASS;
        
        s_u32CHECKSUM = CHECKSUM_SEED;
    }
    else {
        s_u32RomCurrentAddr = u32StartAddr;
        u8TestResult = TEST_PASS;
    }

    return u8TestResult;
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
