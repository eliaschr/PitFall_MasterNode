/********************************************************************************************
 * RTCFuncs.h                                                                               *
 *==========================================================================================*
 *  Created on: Sep 7, 2016                                                                 *
 *      Author: eliaschr                                                                    *
 *******************************************************************************************/

#ifndef RTCFUNCS_H_
#define RTCFUNCS_H_

/********************************************************************************************
 * Includes                                                                                 *
 *******************************************************************************************/
/* Include TI-RTOS header files */
#include <ti/sysbios/knl/Clock.h>

/* Include Standard C header files */
#include <stdint.h>
#include <stdbool.h>


/********************************************************************************************
 * Definitions                                                                              *
 *******************************************************************************************/
#define RTC_DEFTSTAMP		0				//0 Stands for 2000/01/01 00:00:00

#define RTC_MINSECS			60
#define RTC_HOURSECS		3600
#define RTC_DAYSECS			86400
#define RTC_YEARSECS		31536000
#define RTC_QUADYEARDAYS	((4* 365) +1)


/*********************************************************************************************
 * New types definitions                                                                     *
 ********************************************************************************************/
typedef struct {
	uint8_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
} RTCStruct;


/********************************************************************************************
 * Function declarations, functions other code needs to know about                          *
 *******************************************************************************************/
void RTCInit(void);
uint32_t RTCtoTimestamp(char* InDate);
void RTCtoDateStr(uint32_t InTimestamp, char* OutDate, bool flag);
void RTCReset(char* InDate);				//Resets RTC to a defined value
void RTCUpdTm(uint32_t InMin, Clock_Handle UseClk);
uint32_t RTCGetCurrTime(void);
void RTCSetCurrTime(uint32_t InStamp);
void RTCtoStruct(uint32_t InTimeStamp, RTCStruct* OutStr);
uint32_t RTCStructToTstamp(RTCStruct* InStr);

#endif /* RTCFUNCS_H_ */
