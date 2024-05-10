/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     IEC60730_CONTROL_PARAM.h                                                                          *
* @version  V3.00                                                                                             *
* $Revision: 4 $                                                                                             *
* $Date: 21/03/10 3:33p $                                                                                     *
* @brief    IEC60730 User Control Parameters                                                                  *
* @note                                                                                                       *
* Copyright (C) 2013~2015 Nuvoton Technology Corp. All rights reserved.                                       *
***************************************************************************************************************/
#ifndef IEC60730_CONTROL_PARAM_H
#define IEC60730_CONTROL_PARAM_H
#include <stdio.h>
#include "stdint.h"
typedef void (*CLASSB_ERROR_HANDLING)(uint8_t);
typedef void (*CLASSB_WATCHDOG_RESET)(void);
extern uint8_t CLASSB_STARTUP_TESTS(void);
extern void CLASSB_CHECK_RUNTIME_TESTS_EXECUTION(void);
extern void CLASSB_RUNTIME_TESTS(void);
extern void CLASSB_TESTLIB_INIT(CLASSB_ERROR_HANDLING, CLASSB_WATCHDOG_RESET);
extern void CLASSB_LSCLOCK_INT(void);
extern void CLASSB_HSCLOCK_INT(void);
extern uint32_t WriteGlobal(uint32_t *Addr, uint32_t value);
extern uint32_t ReadGlobal(uint32_t *Addr, uint32_t *value);
/* ROM test parameters */
#ifdef __ARMCC_VERSION
extern const volatile uint32_t __Check_Sum;
extern uint32_t Image$$ER_IROM1$$Base;
extern uint32_t Image$$RW_IRAM1$$Base;
extern uint32_t Image$$STACK_PTRN$$Base;
#define ROM_START               ((uint32_t *)&Image$$ER_IROM1$$Base)
#define RAM_START               ((uint32_t *)&Image$$RW_IRAM1$$Base)
#define STACK_START             ((uint32_t *)&Image$$STACK_PTRN$$Base)
#define ROM_CHECKSUM            0x000AF9C4
#endif

#ifdef __ICCARM__
extern const uint32_t __Check_Sum;
extern uint32_t ROM_region$$Base;
extern uint32_t STACK_PTRN$$Base;
#define ROM_START               ((uint32_t *)&RON_region$$Base)
#define RAM_START               0x20000000
#define STACK_START             ((uint32_t *)&STACK_PTRN$$Base)
#define ROM_CHECKSUM            0x0054D0ED
#endif

#define ROM_END                 ((uint32_t *)&__Check_Sum)
#define ROM_LENGTH              ((uint32_t)ROM_END - (uint32_t)ROM_RUNTIME_TEST_LENGTH)
#define CHECKSUM_SEED           0

/* PLL settings */
#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000

/* 
Clock Ratio = (int)(HSCLOCK_FREQ/LSCLOCK_FREQ), MeanError = Clock RatioxClock_Devation, Valid Clock Ration =  Clock Ratio+/-MeanError */
#define HSCLOCK_FREQ        100
#define LSCLOCK_FREQ        34
#define CLOCK_DEVATION      20

/* for Stack test */
#define STACK_OVERRUN_PTRN0     0x12345432
#define STACK_OVERRUN_PTRN1     0x56789876
#define STACK_OVERRUN_PTRN2     0xfedc9876
#define STACK_OVERRUN_PTRN3     0xabcdef12

/* RAM test parameters */
#define RAM_LENGTH              0x00018000

/* Safe State: 0 for IDLE STATE, 1 for RESET STATE */
#define SAFE_STATE              0

/* Debug Message */
#define rt_printf	printf

/* ADC VREF Voltage */
#define VREF_VALUE  3.3
/* Band-Gap Voltage 1.23V threshold 10% */
#define BAND_GAP_H  1679    /* 4095*1.353/VREF_VALUE */
#define BAND_GAP_L  1373    /* 4095*1.107/VREF_VALUE */
/* ADC Mux Test Channel */
#define MUX_CHANNEL     0
#define MUX_CHANNELBIT  BIT0
#define VBG_CHANNEL     7
#define VBG_CHANNELBIT  BIT7

/* each item test cycle */
#define CPUREG_TEST_CYCLE           3       // 3*test_cycle
#define PC_TEST_CYCLE               5       // 5*test_cycle
#define STACK_TEST_CYCLE            10
#define RAM_TEST_CYCLE              5
#define ROM_TEST_CYCLE              1
#define INT_TEST_CYCLE              30
#define REST_WDT_CYCLE              (HSCLOCK_FREQ/LSCLOCK_FREQ + 1)
#define ADC_TEST_CYCLE              4
#define MUX_TEST_CYCLE              4

/* RAM & ROM tested length each time */
#define RAM_RUNTIME_TEST_LENGTH     0x20    // based on Artisan SRAM achitecture
#define ROM_RUNTIME_TEST_LENGTH     0x20

/* RAM runtime test methodology */
#define RAM_RUNTIME_MARCHC_WOM
#ifndef RAM_RUNTIME_MARCHC_WOM
#define RAM_RUNTIME_MARCHX_WOM
#endif

#ifdef __ICCARM__
extern uint32_t GOBAL_AREA$$Base;
extern uint32_t GOBAL_BACK_AREA$$Base;
#define GOBAL_VAR               GOBAL_AREA$$Base
#define GOBAL_BACK_VAR          GOBAL_BACK_AREA$$Base
#endif

#ifdef __ARMCC_VERSION
extern uint32_t Image$$GOBAL_AREA$$Base;
extern uint32_t Image$$GOBAL_BACK_AREA$$Base;
#define GOBAL_VAR               Image$$GOBAL_AREA$$Base
#define GOBAL_BACK_VAR          Image$$GOBAL_BACK_AREA$$Base
#endif

#endif
