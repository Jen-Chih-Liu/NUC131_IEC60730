/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     main.c                                                                                            *
* @version  V1.00                                                                                             *
* $Date: 21/03/09 6:43p $                                                                                     *
* @brief    Main function with STL control loop example                                                       *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include "stdio.h"
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"
extern int32_t system_init(void);

void ClassB_UserErrorHandling(uint8_t error_code)
{
    printf("Error Handling\n");
}
void ClassB_WatchDogReset(void)
{
    SYS_UnlockReg();
    /* reset wdt counter and time out flag */
    WDT_RESET_COUNTER();
    WDT_CLEAR_TIMEOUT_INT_FLAG();
    SYS_LockReg();
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    if(system_init())
        printf("system init - [FAIL]\n");
    else
        printf("system init - [OK]\n");
  
    /* To check if system has been reset by WDT time-out reset or not */
    if(SYS_GetResetSrc() & SYS_RSTSRC_RSTS_WDT_Msk)
    {
        SYS_UnlockReg();
        SYS_ClearResetSrc(SYS_RSTSRC_RSTS_WDT_Msk);
        /* Clear WDT reset flag */
        WDT_CLEAR_RESET_FLAG();
        printf("*** System has been reset by WDT time-out event ***\n\n");
        while(1)
            WDT_RESET_COUNTER();
    }

    /* ClassB Test Lib Init */
    CLASSB_TESTLIB_INIT(ClassB_UserErrorHandling, ClassB_WatchDogReset);
    
    if(CLASSB_STARTUP_TESTS() != 1)
        printf("start up test - [FAIL]\n");
    else
        printf("start up test - [OK]\n");
    
    while (1){
        /* Run Time Execution Check */
        CLASSB_CHECK_RUNTIME_TESTS_EXECUTION();
        /* Run Time Test mainloop */
        CLASSB_RUNTIME_TESTS();
    }
}
