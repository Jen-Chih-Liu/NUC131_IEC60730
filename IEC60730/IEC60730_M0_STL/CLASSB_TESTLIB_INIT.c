/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     CLASSB_TESTLIB_INIT.c                                                                             *
* @version  V1.00                                                                                             *
* $Date: 21/02/08 12:23p $                                                                                     *
* @brief    CLASSB Test Library Initizlizations                                                               *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include "stdio.h"
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"

extern CLASSB_ERROR_HANDLING g_APP_pfnErrorHandling;
extern CLASSB_WATCHDOG_RESET g_APP_pfnWatchDogReset;
extern void INIT_RUNTIME_TESTS(void);
void CLASSB_TESTLIB_INIT(CLASSB_ERROR_HANDLING pfnErrorHandling, CLASSB_WATCHDOG_RESET pfnWatchDogReset)
{
    WriteGlobal((uint32_t *)&g_APP_pfnErrorHandling, (uint32_t)pfnErrorHandling);
    WriteGlobal((uint32_t *)&g_APP_pfnWatchDogReset, (uint32_t)pfnWatchDogReset);
    /* init related variables */
    INIT_RUNTIME_TESTS();
}
