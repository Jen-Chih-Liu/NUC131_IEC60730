/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     ram_march_test.c                                                                                  *
* @version  V1.00                                                                                             *
* $Date: 21/02/08 12:24p $                                                                                     *
* @brief    IEC60730 MarchC method for testing RAM                                                            *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include <string.h>
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"
#include "IEC60730_SAFETY_TEST_LIB.h"
#define SAVE_TEST_RAM    1 
#define RESTORE_TEST_RAM 0

extern uint32_t volatile g_u32RAMStartAddr;
extern uint32_t volatile g_u32RAMEndAddr;
static uint32_t s_u32RamCurrentAddr;
static uint32_t s_u32RamEndAddr;

void IEC60730_Ram_Test_RunTime_Init()
{
    s_u32RamCurrentAddr = (uint32_t) RAM_START;
    s_u32RamEndAddr = (uint32_t) STACK_START;
}
void PreserveTestArea(uint8_t u8move, uint8_t *pu8AddrPtr)
{  
    uint8_t *pu8CopyToSafeArea;
    s_u32RamEndAddr = (uint32_t) STACK_START;

    if (pu8AddrPtr == (uint8_t *)(s_u32RamEndAddr - RAM_RUNTIME_TEST_LENGTH*2))
        pu8CopyToSafeArea= (uint8_t*) (s_u32RamEndAddr - RAM_RUNTIME_TEST_LENGTH);
    else
        pu8CopyToSafeArea= (uint8_t*) (s_u32RamEndAddr - RAM_RUNTIME_TEST_LENGTH*2);

    if(u8move == SAVE_TEST_RAM)
        memcpy(pu8CopyToSafeArea, pu8AddrPtr, RAM_RUNTIME_TEST_LENGTH);
    else
        memcpy(pu8AddrPtr, pu8CopyToSafeArea, RAM_RUNTIME_TEST_LENGTH);
}
uint8_t IEC60730_RAM_MarchC_Test_RunTime()
{
    uint8_t u8TestResult;
    uint8_t *pu8AddrPtr;
    uint32_t u32result, u32value;
    pu8AddrPtr = (uint8_t *) s_u32RamCurrentAddr;

    PreserveTestArea(SAVE_TEST_RAM, pu8AddrPtr);

    /* start ram test */
    
    WriteGlobal((uint32_t *)&g_u32RAMStartAddr, s_u32RamCurrentAddr);
    u32result = ReadGlobal((uint32_t *)&g_u32RAMStartAddr, &u32value);
    if (u32result == 0)
        return TEST_FAIL;
    WriteGlobal((uint32_t *)&g_u32RAMEndAddr, (g_u32RAMStartAddr + RAM_RUNTIME_TEST_LENGTH -1));
    u32result = ReadGlobal((uint32_t *)&g_u32RAMEndAddr, &u32value);
    if (u32result == 0)
        return TEST_FAIL;
    
    u8TestResult = IEC60730_PartRamMarchC_WOM();

    /* restore the data */
    PreserveTestArea(RESTORE_TEST_RAM, pu8AddrPtr);

    /* check if all ram have been tested (excluding safe reserved area) */
    s_u32RamCurrentAddr += RAM_RUNTIME_TEST_LENGTH;
    if (s_u32RamCurrentAddr >= (s_u32RamEndAddr - RAM_RUNTIME_TEST_LENGTH*2))
        IEC60730_Ram_Test_RunTime_Init();
    else
    /* based on Artisan's SRAM architecture, for overlap test */
        s_u32RamCurrentAddr -= RAM_RUNTIME_TEST_LENGTH/2;

    return u8TestResult;
}
