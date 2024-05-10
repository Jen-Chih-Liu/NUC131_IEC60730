/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     stack_overrun_ptrn.c                                                                              *
* @version  V1.00                                                                                             *
* $Date: 21/03/09 4:08p $                                                                                     *
* @brief    IEC60730 Stack Test Patterns                                                                      *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include <stdio.h>
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"

#ifdef __ICCARM__
uint32_t g_u32STACK_TEST_PTRN[] @ "STACK_PTRN" = {STACK_OVERRUN_PTRN0, STACK_OVERRUN_PTRN1,
                                    STACK_OVERRUN_PTRN2, STACK_OVERRUN_PTRN3};
#endif

#ifdef __ARMCC_VERSION
__attribute__((section("STACK_PTRN")))
uint32_t const g_u32STACK_TEST_PTRN[] = {STACK_OVERRUN_PTRN0, STACK_OVERRUN_PTRN1,
                                    STACK_OVERRUN_PTRN2, STACK_OVERRUN_PTRN3};
#endif

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
