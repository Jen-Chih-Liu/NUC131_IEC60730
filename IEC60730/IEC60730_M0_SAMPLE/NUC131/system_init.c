/* ____  _____             ____    ____    _                                 ______    _________   ____  _    *
* |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
*   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
*   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
*  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
* |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
*                                                                                                             *
* @file     system_init.c                                                                                     *
* @version  V1.00                                                                                             *
* $Date: 21/03/09 6:43p $                                                                                     *
* @brief    system initilize                                                                                  *
* @note                                                                                                       *
* SPDX-License-Identifier: Apache-2.0                                                                         *
* @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
***************************************************************************************************************/
#include "stdio.h"
#include "NUC131.h"
#include "IEC60730_CONTROL_PARAM.h"

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
//
    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);
	


    /* Enable HIRC, HXT and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);    
    CLK_EnableXtalRC(CLK_PWRCON_OSC10K_EN_Msk);

    /* Wait for HIRC, HXT and LXT clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC10K_STB_Msk);

    
    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable perpherial module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(WWDT_MODULE);
    CLK_EnableModuleClock(ADC_MODULE);

    /* Select WDT clock source selection from LIRC*/
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_LIRC, 0);

    /* Select WWDT clock source selection from LIRC*/
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL2_WWDT_S_LIRC, 0);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));
		
    /* Select Timer0 module clock source from HIRC*/	
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HIRC, 0);

    /* Select ADC module clock source from HIRC and set ADC module clock divider as 10 */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HIRC, CLK_CLKDIV_ADC(10));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}
/*-------------------------------------------*/
/*                wdt isr                    */
/*-------------------------------------------*/
void WDT_IRQHandler()
{
    /* for wdt startup test */
    SYS_UnlockReg();
    WDT_CLEAR_TIMEOUT_INT_FLAG();
    SYS_LockReg();
    CLASSB_LSCLOCK_INT();
}

/*-------------------------------------------*/
/*                timer0 isr                 */
/*-------------------------------------------*/
void TMR0_IRQHandler(void)
{
    TIMER_ClearIntFlag(TIMER0);
    CLASSB_HSCLOCK_INT();
}

/*-------------------------------------------*/
/*                init timer0                */
/*-------------------------------------------*/
static void InitTimer0(void)
{
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, HSCLOCK_FREQ);

    /* enable TIMER0 Intettupt */
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    TIMER_Start(TIMER0);
}
/*-------------------------------------------*/
/*  init wdt                                 */
/*-------------------------------------------*/
void InitTimer1_WDT(void)
{
    /* enable wdt interrupt */
    NVIC_EnableIRQ(WDT_IRQn);

    /* unlock register */
    SYS_UnlockReg();
#if 1
    /* clear WDT */
    WDT_RESET_COUNTER();
    WDT_CLEAR_TIMEOUT_INT_FLAG();
    
    /* set WDT time out period (1024 ticks), enable WDT, and enable WDT interrupt */
    WDT->WTCR = WDT_TIMEOUT_2POW10 | WDT_WTCR_WTE_Msk | WDT_WTCR_WTIE_Msk | (1 << WDT_WTCR_WTRE_Pos);
    WDT->WTCRALT = WDT_RESET_DELAY_1026CLK;
#endif
    /* lock register */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Systen Initilization Functions                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t system_init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();
    
    /* enable Brown Out Detector and LVR reset */
    SYS_UnlockReg();
	
    SYS -> BODCR |= (SYS_BODCR_BOD_RST_EN | SYS_BODCR_LVR_EN_Msk);
	
    SYS_LockReg();

    InitTimer0();
    InitTimer1_WDT();

    return 0;
}
