/*********************************************************************************************
 * RTCFuncs.c                                                                                *
 *===========================================================================================*
 *  Created on: Sep 7, 2016                                                                  *
 *      Author: eliaschr                                                                     *
 ********************************************************************************************/

/*********************************************************************************************
 * Includes                                                                                  *
 ********************************************************************************************/
/* Include TI-RTOS header files */
#include <ti/sysbios/hal/Hwi.h>

/* Include driverlib files */
#include <driverlib/aon_rtc.h>

/* TI-RTOS Driver files */
#include <ti/drivers/PIN.h>

/* Include other application header files */
#include "Flash/AppFlash.h"
#include "RTC/RTCFuncs.h"

/* Include Standard C Libraries */
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

/* Board files */
#include "Board.h"


/*********************************************************************************************
 * Constants definitions                                                                     *
 ********************************************************************************************/
/*Array that contains the number of seconds passed since the 1st of every month, from the
beginning of the current year.*/
const uint32_t SecsArr[] = {
	0,										//January
	31 *RTC_DAYSECS,						//February
	59 *RTC_DAYSECS,						//March
	90 *RTC_DAYSECS,						//April
	120 *RTC_DAYSECS,						//May
	151 *RTC_DAYSECS,						//June
	181 *RTC_DAYSECS,						//July
	212 *RTC_DAYSECS,						//August
	243 *RTC_DAYSECS,						//September
	273 *RTC_DAYSECS,						//October
	304 *RTC_DAYSECS,						//November
	334 *RTC_DAYSECS						//December
};

/*Array that contais the number of days per month.*/
const uint8_t DaysArr[] = {
	31,										//January
	28,										//February
	31,										//March
	30,										//April
	31,										//May
	30,										//June
	31,										//July
	31,										//August
	30,										//September
	31,										//October
	30,										//November
	31										//December
};


/*********************************************************************************************
 * Variable definitions                                                                      *
 ********************************************************************************************/


/*********************************************************************************************
 * Functions definitions                                                                     *
 ********************************************************************************************/


/*********************************************************************************************
 * Variable definitions                                                                      *
 ********************************************************************************************/
uint32_t RTCSecs;							//The starting seconds value of the RTC
uint32_t RTCFrag;							//The starting fragment value of seconds of RTC
uint32_t RTCOffSecs;						//Offset of RTC from real current timestamp secs
uint32_t RTCOffFrag;						//Offset of RTC from real current timestamp frag.


/*********************************************************************************************
 * Main RTC Functions code                                                                   *
 ********************************************************************************************/

//********************************************************************************************
/*Initializes the Real Time Clock of the CPU. It resets and then enables it.
*/
void RTCInit(void) {
	HWREG(AON_RTC_BASE + AON_RTC_O_SUBSECINC) = *RTCCal;
	RTCReset("2016/01/01 00:00:00");
}


//********************************************************************************************
/*Gets a string in the form of YYYY/MM/DD[space]HH:MM:SS and converts it to timestamp. The
timestamp is the number of seconds passed since Jan 1st, 2000 00:00:00.
Input Parameters:
	InDate: pointer to string that contains the string in the form YYYY/MM/DD[space]HH:MM:SS
		Instead of space character the sequence '%20' can be used just in case the string is
		HTML friendly. The code checks which one is the sequence and skips the necessary
		characters.

Returns:
	The calculated timestamp
*/
uint32_t RTCtoTimestamp(char* InDate) {
	uint32_t CurrYear;
	uint32_t CurrMon;
	uint32_t CurrSec;

	CurrYear = atoi(InDate) -2000;					//Timestamp is calculated from 2000
	InDate += 5;									//InDate points to month
	/*Need to also add the extra days of leap years. Remember that 2000 is also a leap year,
	so, to include it in leap years passed calculation we must add 3 to the current year. The
	idea is simple. If the current year is 2000, there are no full leap years passed yet.
	No extra day should be added in that case (for the calculation of complete years passed
	in seconds). If the current year is 2001, the passed 2000 was a leap year. In that case
	the calculation of number of complete years passed must return 1. Generally, this
	calculation is made by dividing the number of years by 4. That is why we have to add 3 to
	the current year to correctly calculate the number of passed leap years.*/
	CurrSec = ((CurrYear +3) /4) *RTC_DAYSECS;		//Seconds for the passed extra days
													// because of leap years
	CurrSec += CurrYear *RTC_YEARSECS;				//Complete years passed, in seconds
	CurrMon = atoi(InDate) -1;						//Current month, zero based
	InDate += 3;									//Skip month, point to day of month
	if((CurrMon >1) && ((CurrYear %4) == 0)){		//Leap year after February? =>
		CurrSec += RTC_DAYSECS;						// ... then add one more day
	}
	CurrSec += SecsArr[CurrMon];					//Complete months passed, in seconds
	CurrSec += (atoi(InDate) -1) *RTC_DAYSECS;		//Add complete days passed, in secs
	InDate += 2;									//Go to delimiter character
	if(*InDate == ' ') {							//If the delimiter is space,
		InDate++;									// just skip it,
	} else {
		InDate += 3;								//else skip the whole %20
	}
	CurrSec += atoi(InDate) *RTC_HOURSECS;			//Add complete hours, in seconds
	InDate += 3;									//Skip hour
	CurrSec += atoi(InDate) *RTC_MINSECS;			//Add complete minutes, in seconds
	InDate += 3;									//Skip minutes
	CurrSec += atoi(InDate);						//Add seconds passed
	/*CurrSec now contains the number of seconds passed since Jan 01, 2000, 00:00:00*/
	return CurrSec;
}


//********************************************************************************************
/*Converts a timestamp to a string. The timestamp is the number of seconds passed since Jan
1st, 2000 00:00:00. The string is placed in a predefined buffer. The buffer must contain the
propper space to hold the produced string. The string is in the form of
YYYY/MM/DD[space]HH:MM:SS. The space character can be replaced by %20 in order to be HTML
friendly, by setting the input flag to 'true'
Input Parameters:
	InTimestamp: is the timestamp to be converted to string
	OutDate: Points to the string buffer that will receive the resulting string. The buffer
		must have the appropriate space to hold the resulting string, including the
		terminating character '\0'
	flag: If false, the resulting string contains a character between date and time.
		If true, the resulting string contains the sequence "%20" as a delimeter between date
		and time, to make the string HTML friendly
*/
void RTCtoDateStr(uint32_t InTimestamp, char* OutDate, bool flag) {
	RTCStruct CurrTime;

	RTCtoStruct(InTimestamp, &CurrTime);
	sprintf(OutDate, "%4d/%02d/%02d%s%02d:%02d:%02d", CurrTime.Year +2000, CurrTime.Month,
		CurrTime.Day, flag?"%20":" ", CurrTime.Hour, CurrTime.Minute, CurrTime.Second);
}


//********************************************************************************************
/*Resets the value of RTC to a specified timestamp. The timestamp is provided as a string in
the form YYYY/MM/DD[space]HH:MM:SS. The space delimiter between date and time can be the HTML
friendly string "%20" instead.
Input Parameters:
	InDate: is the Date/Time string to be converted to timestamp
*/
void RTCReset(char* InDate) {
	uint32_t tstamp;
	UInt key;

	key = Hwi_disable();					//Disable interrupts. No one should disturb RTC
	/*Need to get the RTC value. We cannot change it, as TI-RTOS resets it on BIOS_start()*/
	RTCSecs = HWREG(AON_RTC_BASE + AON_RTC_O_SEC);
	RTCFrag = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSEC);
	Hwi_restore(key);						//Restore interrupts. Critical section ends
	tstamp = RTCtoTimestamp(InDate);		//Get the timestamp of the input date
	/*Calculate the difference between the RTC value and the timestamp of input date. This is
	the value that we should add to the timestamp to convert it from relative (time elapsed
	from BIOS_start()) to absolute (the value RTC should have).*/
	RTCOffFrag = RTCFrag;
	RTCOffSecs = tstamp -RTCSecs;
}


//********************************************************************************************
/*Sets the comparator channel of RTC to trigger an interrupt after a specified time in
minutes. The use of the RTC channel is to trigger GSM Task to start a signal communication
when it is time to upload data to server
Input Parameters:
	InMin: is the interval in minutes that will wait until firing a GSM Communication event
*/
void RTCUpdTm(uint32_t InMin, Clock_Handle UseClk) {
	UInt key;

	InMin = InMin *60;						//Convert the interval to seconds
	InMin += AONRTCSecGet();
	InMin = (InMin << 16) + (AONRTCFractionGet() >> 16);
	key = Hwi_disable();
	AONRTCCompareValueSet(AON_RTC_CH1, InMin);
	AONRTCChannelEnable(AON_RTC_CH1);
	AONRTCEventClear(AON_RTC_CH1);
	Hwi_restore(key);
}


//********************************************************************************************
/*Gets the current time and returns its calculated timestamp
*/
uint32_t RTCGetCurrTime(void) {
	UInt key;

	key = Hwi_disable();					//Disable interrupts. No one should disturb RTC
	/*Need to get the RTC value. We cannot change it, as TI-RTOS resets it on BIOS_start()*/
	RTCSecs = HWREG(AON_RTC_BASE + AON_RTC_O_SEC);
	RTCFrag = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSEC);
	Hwi_restore(key);						//Restore interrupts. Critical section ends
	return (RTCSecs + RTCOffSecs);			//Return the current timestamp
}


//********************************************************************************************
/*Resets the value of RTC to a specified timestamp. The timestamp is provided as a string in
the form YYYY/MM/DD[space]HH:MM:SS. The space delimiter between date and time can be the HTML
friendly string "%20" instead.
Input Parameters:
	InDate: is the Date/Time string to be converted to timestamp
*/
void RTCSetCurrTime(uint32_t InStamp) {
	UInt key;

	key = Hwi_disable();					//Disable interrupts. No one should disturb RTC
	/*Need to get the RTC value. We cannot change it, as TI-RTOS resets it on BIOS_start()*/
	RTCSecs = HWREG(AON_RTC_BASE + AON_RTC_O_SEC);
	RTCFrag = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSEC);
	Hwi_restore(key);						//Restore interrupts. Critical section ends
	/*Calculate the difference between the RTC value and the timestamp of input date. This is
	the value that we should add to the timestamp to convert it from relative (time elapsed
	from BIOS_start()) to absolute (the value RTC should have).*/
	RTCOffFrag = RTCFrag;
	RTCOffSecs = InStamp -RTCSecs;
}


//********************************************************************************************
/*Converts the input timestamp to calculated structure RTCStruct
*/
void RTCtoStruct(uint32_t CurrTime, RTCStruct* OutStr) {
	OutStr->Second = CurrTime %60;			//Get remaining seconds from complete minutes
	CurrTime = CurrTime /60;				//Timestamp now is in minutes
	OutStr->Minute = CurrTime %60;			//Get remaining minutes from complete hours
	CurrTime = CurrTime /60;				//Timestamp now is in hours
	OutStr->Hour = CurrTime %24;			//Get remaining hours from complete days
	CurrTime = CurrTime /24;				//Timestamp now is in days
	/*In order to avoid extra calculations for leap years, we first calculate the number of
	quad years = 4 Years + 1 day for the leap one.*/
	OutStr->Year = (CurrTime /RTC_QUADYEARDAYS);
	CurrTime = CurrTime %RTC_QUADYEARDAYS;
	/*Now CurrTime contains the number of days of the last quarter of years. So, lets count
	the remaining years. If they are more than two months, then exclude one day, due to the
	fact that the first year of any quarter is the leap one.*/
	OutStr->Day = 0;
	if(CurrTime >= 59) {
		if(CurrTime == 59) {
			OutStr->Day = 1;
		}
		CurrTime--;
	}
	OutStr->Year = (OutStr->Year *4) + (CurrTime /365);
	CurrTime = CurrTime%365;				//Timestamp contains the number of days remain in
											// the current year. The year is handled an a non
											// leap one.
	for(OutStr->Month = 0; (OutStr->Month < 12) && (CurrTime >= DaysArr[OutStr->Month]);
		OutStr->Month++) {
		CurrTime -= DaysArr[OutStr->Month];	//Subtract this month's days
	}
	OutStr->Month++;						//Convert Month to 1 based (1 is for January)
	OutStr->Day = OutStr->Day +CurrTime +1;	//The remaining days are the days in current month
}


//********************************************************************************************
/*Converts the input structure RTCStruct to timestamp. Useful for data packets creation
 */
uint32_t RTCStructToTstamp(RTCStruct* InStr) {
	uint32_t CurrSec;

	/*Need to also add the extra days of leap years. Remember that 2000 is also a leap year,
	so, to include it in leap years passed calculation we must add 3 to the current year. The
	idea is simple. If the current year is 2000, there are no full leap years passed yet.
	No extra day should be added in that case (for the calculation of complete years passed
	in seconds). If the current year is 2001, the passed 2000 was a leap year. In that case
	the calculation of number of complete years passed must return 1. Generally, this
	calculation is made by dividing the number of years by 4. That is why we have to add 3 to
	the current year to correctly calculate the number of passed leap years.*/
	CurrSec = ((InStr->Year +3) /4) *RTC_DAYSECS;	//Seconds for the passed extra days
													// because of leap years
	CurrSec += InStr->Year *RTC_YEARSECS;			//Complete years passed, in seconds
	if((InStr->Month >2) && ((InStr->Year %4) == 0)){	//Leap year after February? =>
		CurrSec += RTC_DAYSECS;						// ... then add one more day
	}
	CurrSec += SecsArr[InStr->Month -1];			//Complete months passed, in seconds
	CurrSec += (InStr->Day -1) *RTC_DAYSECS;		//Add complete days passed, in secs
	CurrSec += InStr->Hour *RTC_HOURSECS;			//Add complete hours, in seconds
	CurrSec += InStr->Minute *RTC_MINSECS;			//Add complete minutes, in seconds
	CurrSec += InStr->Second;						//Add seconds passed
	/*CurrSec now contains the number of seconds passed since Jan 01, 2000, 00:00:00*/
	return CurrSec;
}
