/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     GlobalVar.c                                                                                       *
* @version  V1.00                                                                                             *
* $Date: 21/03/09 4:08p $                                                                                     *
* @brief    Global variable area                                                                              *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include <stdio.h>
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"

extern uint32_t volatile g_ArrayBackup[6];
uint32_t WriteGlobal(uint32_t *Addr, uint32_t value)
{
    uint32_t offset;

    *Addr = value;
    offset = (uint32_t)Addr - ((uint32_t) &GOBAL_VAR);
    *(uint32_t *)((uint32_t)(&GOBAL_BACK_VAR) + offset) = value;	
    return 0;
}

uint32_t ReadGlobal(uint32_t *Addr, uint32_t *value)
{
    uint32_t offset, backup_value;	

    *value = *Addr;
    offset = (uint32_t)Addr - ((uint32_t) &GOBAL_VAR);

    backup_value = g_ArrayBackup[0];
    backup_value = *(uint32_t *)((uint32_t)&GOBAL_BACK_VAR + offset);

    if (*value == backup_value)
        return 1;
    else 
        return 0;
}

//__attribute__((section("GOBAL_AREA")))
uint32_t volatile g_u32RAMStartAddr __attribute__((section("GOBAL_AREA")));
uint32_t volatile g_u32RAMEndAddr __attribute__((section("GOBAL_AREA")));
CLASSB_ERROR_HANDLING g_APP_pfnErrorHandling __attribute__((section("GOBAL_AREA")));
CLASSB_WATCHDOG_RESET g_APP_pfnWatchDogReset __attribute__((section("GOBAL_AREA")));
uint32_t g_u32LSFreq __attribute__((section("GOBAL_AREA")));
uint32_t g_u32HSFreq __attribute__((section("GOBAL_AREA")));
