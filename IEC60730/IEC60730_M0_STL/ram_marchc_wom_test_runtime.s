; ____  _____             ____    ____    _                                 ______    _________   ____  _     *
; |_   \|_   _|           |_   \  /   _|  (_)                              .' ____ \  |  _   _  | |_   _|     *
;   |   \ | |    __   _     |   \/   |    __    .---.   _ .--.    .--.     | (___ \_| |_/ | | \_|   | |       *
;   | |\ \| |   [  | | |    | |\  /| |   [  |  / /'`\] [ `/'`\] / .'`\ \    _.____`.      | |       | |   _   *
;  _| |_\   |_   | \_/ |,  _| |_\/_| |_   | |  | \__.   | |     | \__. |   | \____) |    _| |_     _| |__/ |  *
; |_____|\____|  '.__.'_/ |_____||_____| [___] '.___.' [___]     '.__.'     \______.'   |_____|   |________|  *
;                                                                                                             *
; @file     ram_marchc_wom_test_runtime.s                                                                     *
; @version  V1.00                                                                                             *
; $Date: 21/02/08 12:24p $                                                                                     *                                                       *
; @note                                                                                                       *
; SPDX-License-Identifier: Apache-2.0                                                                         *
; @copyright (C) 2016-2020 Nuvoton Technology Corp. All rights reserved.                                      *
;**************************************************************************************************************
; Description    : Partial RAM MarchC test for runtime
; Input          : R0 .. RAM begin (first address to check), 
;                  R1 .. RAM end (last address to check)
;                  R2 .. Background pattern
; Local          : R3 .. Inverted background pattern
;                  R4 .. keeps test result status
;                  R5  .. pointer to RAM
;                  R6  .. content RAM to compare
; Return         : TEST_SUCCESSFULL (=1)
; WARNING        : The test range of SRAM cannot inclde stack, otherwise may 
;				   resut in unpredicted condition encounterred.
;
	AREA    |.text|, CODE, READONLY

	EXPORT	IEC60730_PartRamMarchC_WOM
	IMPORT	IEC60730_FullRamMarchX_WOM
	IMPORT	g_u32RAMStartAddr
	IMPORT	g_u32RAMEndAddr	
	

;DF: ARTISAN 1
IEC60730_PartRamMarchC_WOM

	PUSH	{R1-R7}
	MOV		R0, R14		
	PUSH	{R0}		; save R14 (LR) register
	BL		IEC60730_FullRamMarchX_WOM
	POP		{R0}
	MOV		R14, R0		; restore R14 (LR) register
	MOV		R0, R4		; pass test result to R0
	POP  	{R1-R7}	
	BX    	LR          ; return to the caller
	
	NOP
	END	
	