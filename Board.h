/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/Power.h>

#include "CC1310DK_7XD.h"

/* Definitions made for ease of PitFall Trap's PCB. DIO_16 and DIO_17 are used for JTAG
 * interface (TDO and TDI, respectively) , so they cannot be used in our project.*/
#define GSM_RxD					IOID_2
#define GSM_TxD					IOID_3
#define	GSM_RTS					IOID_21
#define	GSM_CTS					IOID_22
#define	GSM_DTR					IOID_24		//Used as ACC_CS
#define GSM_DCD					IOID_30		//Used as SDCARD_CS
#define GSM_VCC					IOID_23		//Used as ALS_OUT
#define GSM_PWRKEY				IOID_20		//Used as ACC_PWR
#define	GSM_STATUS				IOID_26		//Used as ALS_PWR
#define GSM_RESET				IOID_1

#define BoardLedR				IOID_25
#define BoardLedY				IOID_27
#define BoardLedG				IOID_7
#define	BoardLedB				IOID_6

#define LCD_3V3					IOID_13
#define LCD_MODE				IOID_4
#define LCD_RESET				IOID_5
#define LCD_CS					IOID_14
#define SPI_SCK					IOID_10
#define SPI_MOSI				IOID_9

#define SPI_MISO				IOID_8

#define BUTTON_L				IOID_15
#define BUTTON_R				IOID_18
#define BUTTON_U				IOID_19
#define BUTTON_D				IOID_12
#define BUTTON_SEL				IOID_11

#define RF_BUTTON				BUTTON_R
#define RF_SYNCLED				BoardLedG
#define RF_ACTIVITYLED			BoardLedY

#define SMS_BUTTON				BUTTON_SEL
#define GSM_SETTINGS			BUTTON_U
#define GSM_ERRORLED			BoardLedR
#define GSM_ACTIVELED			BoardLedB
#define GPRS_ACTIVELED			BoardLedG

#define	Board_ADC0				CC1310DK_7XD_ADCVDDS
#define	Board_ADC1				CC1310DK_7XD_ADCALS

#define	Board_ADCBuf0			CC1310DK_7XD_ADCBuf0
#define	Board_ADCBufChannel0	(0)
#define	Board_ADCBufChannel1	(1)

#define	Board_I2C0				Board_I2C
#define	Board_UART0				Board_UART
#define	Board_AES0				Board_AES
#define	Board_WATCHDOG0			Board_WATCHDOG

#define	Board_initGeneral() { \
    Power_init(); \
    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) \
        {System_abort("Error with PIN_init\n"); \
    } \
}

#define Board_initGPIO()
#define Board_initPWM()			PWM_init()
#define Board_initSPI()			SPI_init()
#define Board_initUART()		UART_init()
#define Board_initWatchdog()	Watchdog_init()
#define Board_initADCBuf()		ADCBuf_init()
#define Board_initADC()			ADC_init()
#define GPIO_toggle(n)
#define GPIO_write(n,m)

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
