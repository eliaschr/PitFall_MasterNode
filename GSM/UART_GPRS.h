/*********************************************************************************************
 * UART_GPRS.h                                                                               *
 *===========================================================================================*
 *  Created on: Aug 18, 2016                                                                 *
 *      Author: eliaschr                                                                     *
 ********************************************************************************************/

#ifndef UART_GPRS_H_
#define UART_GPRS_H_

/*********************************************************************************************
 * Includes                                                                                  *
 ********************************************************************************************/
#include <ti/sysbios/knl/Event.h>			//Events are needed

//Time to retry communication in case of error. It is set in seconds
#define DEFUPDTIME		30					//Default update time at the beginning of the code
#define DEFSETRETRY		120					//Default "Settings" request retry on error:
											// Two minutes
#define DEFSIGRETRY		1800				//Default "Signal" request retry on error:
											// Half an hour

//Lets define the GPRS request type, used by SendSMS function
#define GSMTYPE_SIGNAL		0				//Normal type. Sends all necessary data
#define GSMTYPE_SETTINGS	1				//Settings type. It is used for getting data from
											//server
#define CREG_MAXREPEAT	30					//Maximum number of repetitions of a AT+CREG
											// command until considering the network
											// registration failed
#define CMGL_MAXREPEAT	3					//Maximum number of repetitions of AT+CREG
											// command until considering the serial channel
											// noisy
#define HTTP_MAXREPEAT	3					//Maximum number of repetitions of the HTTP GET
											// request.

//Lets define parameters needed for SMS message filtering
#define SMSMEMSIZE		20					//Length of SMS memory array

/*The next definitions are for SystemStatus variable. They define the bits that are used and
what they represent in the code. They are used to make the code more self-documented*/
#define SS_GSMON		(1<<15)				//Shows if the GSM module is powered on
//#define	SS_RTCOK	(1<<14)				//Shows that the RTC is initialized by the server
#define SS_GPRSFLG		(1<<13)				//Shows if it is necessary to communicate to
											// server, through GPRS
//#define SS_FORCEUP	(1<<12)				//Flags that the system should wake up
#define SS_NEWDATA		(1<<11)				//Flags that there are new data from GSM module
#define SS_GPRSDELAY	(1<<10)				//Flags that the delay set is expired (by timer)
#define SS_GPRSSEROK	(1<<9)				//Flags that we are still in initialization phase
											// of the system. Need to initialize variables
											// from server
#define SS_NOEXIT		(1<<8)				//Flags the need to repeat the state machine
											// without going to sleep first
#define SS_POWEROFF		(1<<7)				//Need to power off the GSM module
#define SS_RESPONSE		(1<<6)				//Flags that a response from a command is expected
#define SS_SECLINE		(1<<5)				//Flags that this is the second line of a responce
#define SS_GSMTYPEREQ	(1<<4)				//Request type. If it is 0 the needed request to
											// be sent is "Settings", else it is "Signal"
#define SS_ALLNOTFOUND	(1<<3)				//Through searching for parameters, it informs
											// that none of the specified parameters found
#define SS_FOUNDALL		(1<<2)				//Through searching for parameters, it informs
											// that all the known parameters are found
#define SS_SMSRECEIVE	(1<<1)				//Indicates that the GSM FSM must fetch new SMSs
#define SS_HTTPSETS		(1<<0)				//Indicates that the GSM FSM should make a
											// Settings request to the server
//#define SS_COMMENDED	(1<<1)				//Flags that the communication is over. Other
											// subsystems can use this flag to initialize
											// themselves

/*The following definitions are for SMSStatus variable. It expresses the current state of the
SMS Reading state machine*/
#define SMS_ENDFOUND	(1<<16)				//Flags the presentation of a line ending when
											// searching the GSM reception buffer
#define SMS_MORE		(1<<15)				//Expresses there are more SMS in list than the
											// SMS array can hold
#define SMS_SHORTDELAY	(1<<14)				//Defines the delay when expecting new incoming
											// messages
#define SMS_NEEDDELAY	(1<<13)				//Expresses if we need a delay when garbage
											// characters are detected during PDU reading
#define SMS_APNSET		(1<<12)				//Need to update APN in flash memory
#define SMS_USERSET		(1<<11)				//Need to update GPRS username in flash memory
#define SMS_PASSSET		(1<<10)				//Need to update GPRS password in flash memory
#define SMS_HOSTSET		(1<<9)				//Need to update internet host server
#define SMS_PORTSET		(1<<8)				//Need to update host server's communication port
#define SMS_UIDSET		(1<<7)				//Need to update the UID of this master node
#define SMS_CALSET		(1<<6)				//Need to update the RTC calibration value
#define SMS_NODTMSET	(1<<5)				//Need to update the interval between RF actions
#define SMS_UIDRESET	(1<<4)				//Need to reset the UID to its default value
#define SMS_UCS2		(1<<3)				//Flags that the message is UCS2 encoded
#define SMS_8BIT		(1<<2)				//Flags that the message is 8 bit encoded
											//In none of the two above is set then the message
											// is 7 bit encoded
#define SMS_FAIL		(1<<1)				//Flags that there was a failure due to garbage
											// characters
#define SMS_DISCARD		(1<<0)				//Flags that we need to discard the rest of PDU
											// data of the currently read SMS
//Define bit masks for both Flash Information Segment D and C (MSP430)
#define SMS_FLASHMASKD	(SMS_APNSET | SMS_USERSET | SMS_PASSSET)
#define SMS_FLASHMASKC	(SMS_HOSTSET | SMS_PORTSET | SMS_UIDSET | SMS_CALSET | SMS_NODTMSET)
//Also define a generic flash bit mask
#define SMS_FLASHMASK	(SMS_FLASHMASKC | SMS_FLASHMASKD)

/*The following definitions are for GsmFsmState variable. It expresses the current state of
the GSM state machine. It is used to control the steps to perform in order to communicate to
server or Mobile Service Provider for SMS checking*/
#define GFSM_OFF		0					//GSM module is totally off. It is not in use
#define GFSM_MODULEPWR	1					//Shows that the GSM module is powered on
#define GFSM_PBPUSHED	2					//Power button is pressed
#define GFSM_PBRELEASED	3					//Power button just released
#define GFSM_SEROK		4					//Serial port is ready
#define GFSM_SETAT		5					//Sends a dummy AT command for the autobaudrate
#define GFSM_SETECHO	6					//Stop echoing of characters
#define GFSM_FLOWCTL	7					//Enables RTS/CTS flow control for both Rx and Tx
#define GFSM_ASKIMEI	8					//Asks the IMEI from the module
#define GFSM_COMMOK		9					//Communication established. OK
#define GFSM_GETRSSI	10					//Get the RRSI of the communication

#define GFSM_ATTACH		50					//Attach to GPRS service
#define GFSM_DEFPDP		51					//Define PDP context
#define GFSM_STTASK		52					//Start task and set APN, user, password
#define GFSM_CONNON		53					//Bring GPRS communication on
#define GFSM_GETIP		54					//Get local IP
#define GFSM_ADDIPHEAD	55					//Add an IP Header to data
#define GFSM_STARTTCP	56					//Start a TCP/IP connection
#define GFSM_PREPDATA	57					//Prepare data request to be sent
#define GFSM_INSDATA	58					//Construct the whole HTTP request
#define GFSM_FINDIPD	59					//Find "+IPD,<length>:" responce
#define GFSM_GETLENGTH	60					//Filter <length> parameter
#define GFSM_GETDATA	61					//Set the lentgh of data to be read
#define GFSM_PARSEDATA	62					//Parse incomming data
#define GFSM_CLOSE		63					//Closes the TCP connection to the server
#define GFSM_DEACT		64					//Deactivates the PDP context

#define GFSM_SETTXTSMS	100					//Set SMS in Text Mode
#define GFSM_SETPDUSMS	101					//Set SMS in PDU mode
#define GFSM_SMSLIST	102					//Get stored SMS list and get their lengths
#define GFSM_WAITNEXT	103					//Waits for a short time (several seconds) for the
											// possibility of an incoming SMS
#define GFSM_READSMS	104					//Reads a list of SMSs
#define GFSM_DELALLSMS	105					//Delete ALL READ SMSs
#define GFSM_DELSMS		106					//Delete a single SMS (current index)

#define GFSM_POWEROFF	253					//Going to power off the module
#define GFSM_POFFPUSH	254					//Power button pressed to power off the module
#define GFSM_POFFREL	255					//Power button released to power off the module

/*The following definitions are for GSMSerialStatus variable. They define the bits that are
used and what they represent in the code. They are used to make the code more self-documented
*/
#define GSMS_BINMODE	(1<<15)				//Flags that GSM reception is in binary mode (no
											// newline character sequences filtering)
#define GSMS_0ACHAR		(1<<14)				//Flags the reception of a 0Ah character
#define GSMS_0DCHAR		(1<<13)				//Flags the reception of a 0Dh character
#define GSMS_NEWLINE	(1<<12)				//Flags the reception of a full \n
#define GSMS_TIMED		(1<<11)				//Flags that the communication should be timed out
											// if there is no character received within a time
											// limit
#define GSMS_WAITINP	(1<<10)				//Expected answers are not terminated lines but a
											// partial string, like AT+CIPSEND command or
											// server answers. This flag instructs the Serial
											// FSM to just wait for a specific series of
											// characters. If found the answer lies at the
											// GsmCmpCounter variable, but also in GSMS_FOUND
											// flag of GSMSerialStatus
#define GSMS_INIT		(1<<9)				//It is similar to WAITINP but it does not stop
											// the communication in case of a different
											// character than expected. It wait until it finds
											// the full string specified by GsmCmpPtr
#define GSMS_FOUND		(1<<8)				//Flags if the expected string is found (when the
											// GSMS_WAITINP flag is 1)
#define GSMS_INTEGER	(1<<7)				//Expects incoming data to be ascii number. The
											// serial state machine stores the number in an
											// integer variable
#define GSMS_FORCESTORE	(1<<6)				//Internal flag that forces RS232 Rx state machine
											// to store the last character in reception buffer
#define GSMS_COUNTER	(1<<5)				//Instructs the serial state machine to countdown
											// the received characters and inform when the
											// limit is reached. GsmCmpCounter defines the
											// characters limit
#define GSMS_LIMITED	(1<<4)				//Informs that serial port reached the limit of
											// characters in a GSMS_LIMITED reception
#define GSMS_LIMITREAD	(1<<3)				//Expresses that the character of the specified
											// limit is read (used with GSMS_COUNTER and
											// GSMS_LIMITED flags)
#define GSMS_LIMITACK	(1<<2)				//Acknowledges the LIMITED flag
#define GSMS_STATUS		(1<<1)				//Expresses that Status pin is 1
#define GSMS_CTS		(1<<0)				//Expresses that CTS pin is 0

/*For the Event object that notifies the GPRS task for various events, the following flags
are used*/
#define EV_GPRSFLG		Event_Id_13			//Shows if it is necessary to communicate to
											// server, through GPRS
#define EV_GPRSSEROK	Event_Id_09			//Shows that the serial port of GSM module is
											// ready to accept communication
#define EV_HTTPSIG		Event_Id_04			//Event to start a GPRS Signal request
#define EV_SMSRECEIVE	Event_Id_01			//Event to start GSM and receive SMS messages
#define EV_HTTPSETS		Event_Id_00			//Event to start a GPRS Settings request
//Lets create a macro for all those events able to fire up a GSM communication
#define EV_GPRSALL		(EV_GPRSFLG | EV_HTTPSETS | EV_SMSRECEIVE | EV_HTTPSIG)

/*********************************************************************************************
 * Extern declarations, variables needed by other tasks                                      *
 ********************************************************************************************/
//The GSM Semaphore handle. This semaphore is used to fire up a GPRS communication
extern Event_Handle GPRSEvt;				//Global to all files that need to post one
extern long int UpdateTime;					//Holds the update interval since our last server
extern volatile uint32_t ClkEvent;			//The event that is passed to the clock callback
extern volatile uint32_t SystemStatus;		//Status flags for keeping the status of the FSM


/*********************************************************************************************
 * Function declarations, functions other code needs to know about                           *
 ********************************************************************************************/
void InitGPRSTask(void);					//Initializes the GPRS task
void GSMRestart(UArg InArg);				//Clock function to retrigger GPRS communication

#endif /* UART_GPRS_H_ */
