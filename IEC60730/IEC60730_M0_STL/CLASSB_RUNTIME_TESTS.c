/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     CLASSB_RUNTIME_TEST.c                                                                             *
* @version  V1.00                                                                                             *
* $Date: 21/03/09 4:08p $                                                                                     *
* @brief    CLASSB Run Time tests                                                                             *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"
#include "IEC60730_SAFETY_TEST_LIB.h"

static volatile uint32_t s_u32HSCounter = 0;
RunTimeTestCounter_T static RunTimeTestCounter;
RunTimeTestExecution_T static RunTimeTestExecution;

#define FUNC_REGISTER_TEST       0
#define FUNC_PROGRAMCOUNTER_TEST 1
#define FUNC_STACK_TEST          2
#define FUNC_RAM_TEST            3
#define FUNC_ROM_TEST            4
#define FUNC_WATCHDOG_RESET      5
#define FUNC_ADC_TEST            6
#define FUNC_MUX_TEST            7
#define FUNC_INTERRUPT_TEST      8
extern uint8_t CLASSB_SAFE_STATE(uint8_t TestResult);
extern uint8_t CLASSB_WatchDog_Reset(void);
extern void IEC60730_Flash_Test_RunTime_Init(void);
extern void IEC60730_Ram_Test_RunTime_Init(void);
extern void IEC60730_Interrupt_Test_RunTime_Init(void);
extern void IEC60730_Stack_Test_RunTime_Init(void);
extern void IEC60730_LSCLOCK_INIT(void);
extern void IEC60730_HSCLOCK_INIT(void);
extern void IEC60730_LSCLOCK_INT(void);
extern void IEC60730_HSCLOCK_INT(void);
void INIT_RUNTIME_TEST_COUNTER(void)
{
    RunTimeTestCounter.REGISTER_TEST = s_u32HSCounter;
    RunTimeTestCounter.PROGRAMCOUNTER_TEST = s_u32HSCounter;
    RunTimeTestCounter.STACK_TEST = s_u32HSCounter;
    RunTimeTestCounter.ROM_TEST = s_u32HSCounter;
    RunTimeTestCounter.RAM_TEST = s_u32HSCounter;
    RunTimeTestCounter.WATCHDOG_RESET = s_u32HSCounter;
    RunTimeTestCounter.ADC_TEST = s_u32HSCounter;
    RunTimeTestCounter.MUX_TEST = s_u32HSCounter;
    RunTimeTestCounter.INTERRUPT_TEST = 0;

    RunTimeTestExecution.REGISTER_TEST = 0;
    RunTimeTestExecution.PROGRAMCOUNTER_TEST = 0;
    RunTimeTestExecution.STACK_TEST = 0;
    RunTimeTestExecution.ROM_TEST = 0;
    RunTimeTestExecution.RAM_TEST = 0;
    RunTimeTestExecution.WATCHDOG_RESET = 0;
    RunTimeTestExecution.ADC_TEST = 0;
    RunTimeTestExecution.MUX_TEST = 0;
    RunTimeTestExecution.INTERRUPT_TEST = 0;
}
void CLASSB_HSCLOCK_INIT(void)
{
    s_u32HSCounter = 0;
    IEC60730_HSCLOCK_INIT();
}
void CLASSB_LSCLOCK_INT(void)
{
    IEC60730_LSCLOCK_INT();
    RunTimeTestCounter.INTERRUPT_TEST = 1;
}
void CLASSB_HSCLOCK_INT(void)
{
    s_u32HSCounter++;
    IEC60730_HSCLOCK_INT();
    /* Overflow handling */
    if(s_u32HSCounter==0) { 
        INIT_RUNTIME_TEST_COUNTER();
    }
}
void INIT_RUNTIME_TESTS(void)
{
    INIT_RUNTIME_TEST_COUNTER();
    IEC60730_Stack_Test_RunTime_Init();
    IEC60730_Flash_Test_RunTime_Init();
    IEC60730_Ram_Test_RunTime_Init();
    IEC60730_Interrupt_Test_RunTime_Init();
    CLASSB_HSCLOCK_INIT();
}
void CLASSB_CHECK_RUNTIME_TESTS_EXECUTION()
{
    /* Check go execution or not in this turn */
    if(s_u32HSCounter>=RunTimeTestCounter.REGISTER_TEST)
        RunTimeTestExecution.REGISTER_TEST = 1;
    else
        RunTimeTestExecution.REGISTER_TEST = 0;

    if(s_u32HSCounter>=RunTimeTestCounter.PROGRAMCOUNTER_TEST)
        RunTimeTestExecution.PROGRAMCOUNTER_TEST = 1;
    else
        RunTimeTestExecution.PROGRAMCOUNTER_TEST = 0;

    if(s_u32HSCounter>=RunTimeTestCounter.STACK_TEST)
        RunTimeTestExecution.STACK_TEST = 1;
    else
        RunTimeTestExecution.STACK_TEST = 0;

    if(s_u32HSCounter>=RunTimeTestCounter.ROM_TEST)
        RunTimeTestExecution.ROM_TEST = 1;
    else
        RunTimeTestExecution.ROM_TEST = 0;

    if(s_u32HSCounter>=RunTimeTestCounter.RAM_TEST)
        RunTimeTestExecution.RAM_TEST = 1;
    else
        RunTimeTestExecution.RAM_TEST = 0;

    if(s_u32HSCounter>=RunTimeTestCounter.WATCHDOG_RESET)
        RunTimeTestExecution.WATCHDOG_RESET = 1;
    else
        RunTimeTestExecution.WATCHDOG_RESET = 0;

    if(s_u32HSCounter>=RunTimeTestCounter.ADC_TEST)
        RunTimeTestExecution.ADC_TEST = 1;
    else
        RunTimeTestExecution.ADC_TEST = 0;

    if(s_u32HSCounter>=RunTimeTestCounter.MUX_TEST)
        RunTimeTestExecution.MUX_TEST = 1;
    else
        RunTimeTestExecution.MUX_TEST = 0;

    if(RunTimeTestCounter.INTERRUPT_TEST)
        RunTimeTestExecution.INTERRUPT_TEST = 1;
    else
        RunTimeTestExecution.INTERRUPT_TEST = 0;



    /* Update counter by test cycle */
    if(RunTimeTestExecution.REGISTER_TEST == 1)
        RunTimeTestCounter.REGISTER_TEST = s_u32HSCounter + CPUREG_TEST_CYCLE;
    if(RunTimeTestExecution.PROGRAMCOUNTER_TEST == 1)
        RunTimeTestCounter.PROGRAMCOUNTER_TEST = s_u32HSCounter + PC_TEST_CYCLE;
    if(RunTimeTestExecution.STACK_TEST == 1)
        RunTimeTestCounter.STACK_TEST = s_u32HSCounter + STACK_TEST_CYCLE;
    if(RunTimeTestExecution.ROM_TEST == 1)
        RunTimeTestCounter.ROM_TEST = s_u32HSCounter + ROM_TEST_CYCLE;
    if(RunTimeTestExecution.RAM_TEST == 1)
        RunTimeTestCounter.RAM_TEST = s_u32HSCounter + RAM_TEST_CYCLE;
    if(RunTimeTestExecution.WATCHDOG_RESET == 1)
        RunTimeTestCounter.WATCHDOG_RESET = s_u32HSCounter + REST_WDT_CYCLE;
    if(RunTimeTestExecution.ADC_TEST == 1)
        RunTimeTestCounter.ADC_TEST = s_u32HSCounter + ADC_TEST_CYCLE;
    if(RunTimeTestExecution.MUX_TEST == 1)
        RunTimeTestCounter.MUX_TEST = s_u32HSCounter + MUX_TEST_CYCLE;
    if(RunTimeTestExecution.INTERRUPT_TEST == 1)
        RunTimeTestCounter.INTERRUPT_TEST = 0;
}
void CLASSB_RUNTIME_TESTS(void)
{
    uint8_t u8TestResult;
    /* reset wdt */
    if(RunTimeTestExecution.WATCHDOG_RESET == 1) {
        u8TestResult = CLASSB_WatchDog_Reset();
        if(u8TestResult)
            rt_printf("Watch dog counter reset !!\n");
    }
    
    /* CPU Registers Test */
    if(RunTimeTestExecution.REGISTER_TEST == 1) 
    {
        u8TestResult = CLASSB_Registers_Test(RUNTIME);
        if(u8TestResult)
            rt_printf("runtime CPU reg test: Pass !!\n");
        else
            CLASSB_SAFE_STATE(CPU_TEST_FAIL);
    }

    /* Program Counter Test */
    if(RunTimeTestExecution.PROGRAMCOUNTER_TEST == 1) {
        u8TestResult = CLASSB_ProgramCounter_Test();
        if(u8TestResult)
            rt_printf("runtime PC test: Pass !!\n");
        else
            CLASSB_SAFE_STATE(PC_TEST_FAIL);
    }

    /* Stack Overrun Test */
    if(RunTimeTestExecution.STACK_TEST == 1) {
        u8TestResult = CLASSB_Stack_Test();
        if(u8TestResult)
            rt_printf("runtime Stack test: Pass !!\n");
        else
            CLASSB_SAFE_STATE(STACK_TEST_FAIL);
    }

    /* RAM Test */
    if(RunTimeTestExecution.RAM_TEST == 1) {
        u8TestResult = CLASSB_RAM_Test(RUNTIME);
        if(u8TestResult)
            rt_printf("runtime RAM test: Pass !!\n");
        else
            CLASSB_SAFE_STATE(RAM_TEST_FAIL);
    }

    /* ROM Test */
    if(RunTimeTestExecution.ROM_TEST == 1) {
        u8TestResult = CLASSB_Flash_Test(RUNTIME);
        if(u8TestResult)
            rt_printf("runtime ROM test: Pass !!\n");
        else 
            CLASSB_SAFE_STATE(FLASH_TEST_FAIL);
    }
    
    /* ADC test */
    if(RunTimeTestExecution.ADC_TEST == 1) {
        u8TestResult = CLASSB_ADC_Test(RUNTIME);
        if(u8TestResult)
            rt_printf("runtime ADC test: Pass !!\n");
        else 
            CLASSB_SAFE_STATE(ADC_TEST_FAIL);
    }
    
    /* MUX test */
    if(RunTimeTestExecution.MUX_TEST == 1) {
        u8TestResult = CLASSB_MUX_Test(RUNTIME);
        if(u8TestResult)
            rt_printf("runtime MUX test: Pass !!\n");
        else 
            CLASSB_SAFE_STATE(MUX_TEST_FAIL);
    }
    
    /* Interrupt/Clock test */
    if(RunTimeTestExecution.INTERRUPT_TEST) {
        u8TestResult = CLASSB_Interrupt_Clock_Test();
        if(u8TestResult)
            rt_printf("runtime Interrupt/Clock test: Pass !!\n");
        else 
            CLASSB_SAFE_STATE(INTERRUPT_TEST_FAIL);
    }
}
/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
