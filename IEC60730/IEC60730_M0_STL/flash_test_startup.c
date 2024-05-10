/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     flash_test_startup.c                                                                              *
* @version  V1.00                                                                                             *
* $Date: 21/03/09 4:08p $                                                                                    *
* @brief    IEC60730 Intrnal Flash Test startup time                                                          *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"
#include "IEC60730_SAFETY_TEST_LIB.h"

#ifdef __ICCARM__
//User Checksum - must be placed at the end of memory
const uint32_t __Check_Sum__attribute__((section("CHECKSUM"))) = ROM_CHECKSUM;
#endif

#ifdef __ARMCC_VERSION
//User Checksum - must be placed at the end of memory
const volatile uint32_t __Check_Sum __attribute__((section("CHECKSUM"))) = ROM_CHECKSUM;
#endif

uint32_t checksum_flash(uint8_t *string, uint32_t u32seed, uint32_t u32Length)
{
    uint32_t  i;
    uint32_t  u32Checksum = u32seed;

    uint8_t  u8FlashContext;

    for (i = 0; i < u32Length; i++)
    {
        u8FlashContext = string[i];
        u32Checksum = u32Checksum + u8FlashContext;
    }

    return u32Checksum;
}

uint8_t IEC60730_Flash_Test()
{
    uint32_t    u32StartAddr;
    uint8_t     u8TestResult;
    volatile uint32_t u32CHECKSUM;

    u32StartAddr = (uint32_t)ROM_START;

    u32CHECKSUM = checksum_flash((uint8_t*)u32StartAddr,CHECKSUM_SEED, (uint32_t)ROM_LENGTH);

    /* compare checksum value */
    if(u32CHECKSUM != __Check_Sum)
        u8TestResult = TEST_FAIL;
    else
        u8TestResult = TEST_PASS;

    return u8TestResult;
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
