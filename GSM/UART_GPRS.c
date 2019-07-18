/*********************************************************************************************
 * UART_GPRS.c                                                                               *
 *===========================================================================================*
 *  Created on: Aug 18, 2016                                                                 *
 *      Author: eliaschr                                                                     *
 ********************************************************************************************/

/*********************************************************************************************
 * Includes                                                                                  *
 ********************************************************************************************/
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Driver files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include <inc/hw_aon_rtc.h>

/* Board Header files */
#include "Board.h"

/* Include other application header files */
#include "PitFall_MasterNode.h"
#include "GSM/UART_GPRS.h"
#include "Flash/AppFlash.h"
#include "GenericFuncs.h"
#include "RTC/RTCFuncs.h"
#include "RF/RadioTask.h"
#include "MainTask.h"

/* Include standard C libraries */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


/*********************************************************************************************
 * Definitions to control the task behavior                                                  *
 ********************************************************************************************/
/*In Debug State, there is a GSMFSMStack Array, that holds the history of GSM FSM states,
SystemStatus variable and GSMSerialStatus variable. For the release code comment this line out
*/
/*eliaschr@INFO: DEBUGFSM is set in preprocessor definitions (currently set in Debug mode
only and not in Release).*/
//#define DEBUGFSM							//When used, the program enters a debugging state
#define DEBUGFSMSTACKSIZE	64				//Sets the length of the GSM FSM debug stack

#define GPRSSTACKSIZE   	1024			//Stack size of GPRS task
#define GPRSPRIORITY		1
#define GSMINSIZE			128				//Size of buffer that holds the incoming GSM data
#define SMSMEMSIZE			20				//Length of SMS memory array

/*The following definitions are for helping string parsing and parameter searching state
machine*/
#define SRCH_SKIPWHITE		0				//Skipping white characters
#define SRCH_SEARCH			1				//Searching for a parameter name
#define SRCH_EXEC			2				//Calling found parameter's associated function
#define SRCH_SKIPLETTER		3				//Skipping normal letters


/*********************************************************************************************
 * Definitions to control the whole code behaviour                                           *
 ********************************************************************************************/
#ifndef DEBUGFSM
#define	DEBUGENTER(name)
#define DEBUGEXIT(path)
#endif

#define STAMPTHRESHOLD		20				//Threshold of timestamps in buffer where the
											// system starts sending the timestamps to server

/*********************************************************************************************
 * New types definitions                                                                     *
 ********************************************************************************************/
#ifdef DEBUGFSM
typedef struct {				// Structure design for GSM State Machine Stack
	int	State;					// Keeps the current state number
	int	RealState;				// Keeps the real state that is entered
	unsigned long int InTime;	// Keeps the time tick number the state triggered
	unsigned int InSysStatus;	// Keeps the current system status variable during state entrance
	unsigned int InSerStatus;	// Keeps the current serial port status during state entrance
//	int InWakeTimes;			// Keeps the current GSMSWakeTimes variable during state entrance
	unsigned int OutSysStatus;	// Keeps the current system status variable on state exit
	unsigned int OutSerStatus;	// Keeps the current serial port status on state exit
//	int OutWakeTimes;			// Keeps the current GSMSWakeTimes variable on state exit
	int ExitPath;				// Keeps the path of the exit from the state
} strGSMStack;
#endif

typedef struct {			// Structure to help reading of SMS messages
  uint8_t index;			// Memory index the SMS is stored
  uint8_t length;			// Length of SMS in characters
} strSMSMem;

typedef struct {			// Structure to help searching through all parameters at once
  int8_t offset;			// Offset to current character in current searched parameter
  int8_t status;			// Instruct the system if the parameter is in search, found or not
} strParamSearch;
#define PS_SCAN			0	// Still trying to find if this is the parameter
#define PS_FOUND		1	// Parameter found
#define PS_NOTFOUND		-1	// Parameter not found

typedef struct {			// Structure to help associating parameters to functions to call
  							//  when a parameter is met
  char* str;				// String offset of parameter
  int8_t (*func)(int, uint8_t);	// Pointer to function that reads and sets the parameter
} strParamFunc;

typedef struct {			// Structure to help associating parameters to functions to call
  							//  when a parameter is met
  char* str;				// String offset of parameter
  int8_t (*func)(int);		// Pointer to function that reads and sets the parameter
} strSMSParamFunc;


/*********************************************************************************************
 * Function declarations                                                                     *
 ********************************************************************************************/
#ifdef DEBUGFSM
void DEBUGENTER(int name);					//Stores the entry of a state in debug FSM stack
void DEBUGEXIT(int path);					//Stores the exit from a state in debug FSM stack
#endif

/* Callback functions */
void cbGSMStatus(PIN_Handle handle, PIN_Id pinId);	//Callback function for tracking Status
											// and CTS pins, if CTS is not used by hardware
//void SetGPRSDelay(uint32_t delay);		//Waits for an event, either timeout, or OK

/* State Machine's helper functions */
void FSMEnterPowerOff(void);				//Forces the state machine to enter power off
											// state

/* Servers response handling */
int8_t SuccessComm(int offset, uint8_t NodeAddr);	//Called when there is HTTP 200 OK
int8_t OKParse(int offset, uint8_t NodeAddr);		//Called when there is an OK reply
int8_t FalseParse(int offset, uint8_t NodeAddr);	//Called when there is a FALSE reply
int8_t UpdTmParse(int offset, uint8_t NodeAddr);	//Handles UpdTm parameter
int8_t NowTmParse(int offset, uint8_t NodeAddr);	//Handles NowTm parameter
int8_t ResetParse(int offset, uint8_t NodeAddr);	//Handles Reset parameter
int8_t SavedParse(int offset, uint8_t NodeAddr);	//Handles StmpOK parameter

/* SMS parameters handling */
int8_t APNParse(int offset);				//Handles APN parameter
int8_t UserParse(int offset);				//Handles Username parameter
int8_t PassParse(int offset);				//Handles Password parameter
int8_t HostParse(int offset);				//Handles Host parameter
int8_t PortParse(int offset);				//Handles Port parameter
int8_t UIDParse(int offset);				//Handles UID parameter
int8_t ResUIDParse(int offset);				//Handles ResetUID parameter (command)
int8_t RTCCalParse(int offset);				//Handles RTCCal parameter
int8_t NodeUpdParse(int offset);			//Handles NodeUpdate parameter

/* Other helper functions */
//Extract the value of a parameter in a SMS
int8_t ParseValue(char* InStr, char* OutStr, int OutLength, int Flag);

/* Main task function */
Void GPRSTaskFxn(UArg arg0, UArg arg1);		//Task function


/*********************************************************************************************
 * Constants definitions                                                                     *
 ********************************************************************************************/
//Table of pins used. It is constant in Flash memory
const PIN_Config GSMPinTable[] = {
/* The commented out lines are for reminding the use of those pins as special signals */
//	GSM_RxD		| PIN_GPIO_OUTPUT_EN| PIN_GPIO_LOW	| PIN_PUSHPULL	| PIN_DRVSTR_MIN,
//	GSM_TxD		| PIN_GPIO_OUTPUT_EN| PIN_GPIO_LOW	| PIN_PUSHPULL	| PIN_DRVSTR_MIN,
//	GSM_RTS		| PIN_GPIO_OUTPUT_EN| PIN_GPIO_LOW	| PIN_PUSHPULL	| PIN_DRVSTR_MIN,
//	GSM_CTS		| PIN_INPUT_EN		| PIN_PULLUP	| PIN_IRQ_NEGEDGE,
	GSM_DTR		| PIN_GPIO_OUTPUT_EN| PIN_GPIO_LOW	| PIN_PUSHPULL	| PIN_DRVSTR_MIN,
	GSM_DCD		| PIN_INPUT_EN		| PIN_PULLUP,
	GSM_VCC		| PIN_GPIO_OUTPUT_EN| PIN_GPIO_LOW	| PIN_PUSHPULL	| PIN_DRVSTR_MIN,
	GSM_PWRKEY	| PIN_GPIO_OUTPUT_EN| PIN_GPIO_LOW	| PIN_PUSHPULL	| PIN_DRVSTR_MIN,
//	GSM_STATUS	| PIN_INPUT_EN		| PIN_PULLDOWN	| PIN_IRQ_POSEDGE,
	GSM_STATUS	| PIN_INPUT_EN		| PIN_PULLDOWN	| PIN_IRQ_DIS,
	GSM_RESET	| PIN_GPIO_OUTPUT_EN| PIN_GPIO_LOW	| PIN_PUSHPULL	| PIN_DRVSTR_MIN,
    PIN_TERMINATE
};

//The following table contains the commands to be sent to the GSM module through UART
const char* GSM_Commands[] = {
	"AT\r\n",								//Dummy AT helps GSM auto discover the baud rate
	"ATE0\r\n",								//Disables the ECHO of GSM serial port
	"AT+IFC=2,2\r\n",						//Enable hardware flow control, both ways
	"AT+GSN\r\n",							//Get IMEI
	"AT+CREG?\r\n",							//Get network registration status of GSM module
	"AT+CSQ\r\n",							//Check signal quality (RSSI, BER)
	"AT+CMGF=\r\n",							//Set SMS to PDU or Text mode. Before the \r\n
											// sequence there must be a number of 0 or 1
	"AT+CMGL=4,1\r\n",						//List SMSs without changing status to "READ"
	"AT+CMGR=\r\n",							//Read one message from SIM Card's memory
	"AT+CMGD=0,3\r\n",						//Delete all (or one) messages
	"AT+CGATT=1\r\n",						//Attach to GPRS service
	"AT+CGDCONT=1,\"IP\",\"\r\n",			//Define PDP to be used by GSM module
	"AT+CSTT=\",\"\r\n",					//Start GPRS Task
	"AT+CIICR\r\n",							//Bring Up GPRS Connection
	"AT+CIFSR\r\n",							//Get IP address
	"AT+CIPHEAD=1\r\n",						//Add IP Header
	"AT+CIPSTART=\"TCP\",\"\r\n",			//Start TCP/IP Communication
	"AT+CIPSEND\r\n",						//Initiate HTTP request
	"AT+CIPCLOSE=1\r\n",					//Close current TCP connection
	"AT_CIPSHUT\r\n"						//Shutdown the PDP context
};
//Definitions of indexes of GSM commands for easier handling and self documented code
#define GSM_CMD_AT			0
#define GSM_CMD_ATE0		(GSM_CMD_AT +1)
#define GSM_CMD_IFC			(GSM_CMD_ATE0 +1)
#define GSM_CMD_GSN			(GSM_CMD_IFC +1)
#define GSM_CMD_CREG		(GSM_CMD_GSN +1)
#define GSM_CMD_CSQ			(GSM_CMD_CREG +1)
#define GSM_CMD_CMGF		(GSM_CMD_CSQ +1)
#define GSM_CMD_CMGL		(GSM_CMD_CMGF +1)
#define GSM_CMD_CMGR		(GSM_CMD_CMGL +1)
#define GSM_CMD_CMGD		(GSM_CMD_CMGR +1)
#define GSM_CMD_CGATT		(GSM_CMD_CMGD +1)
#define GSM_CMD_CGDCONT		(GSM_CMD_CGATT +1)
#define GSM_CMD_CSTT		(GSM_CMD_CGDCONT +1)
#define GSM_CMD_CIICR		(GSM_CMD_CSTT +1)
#define GSM_CMD_CIFSR		(GSM_CMD_CIICR +1)
#define GSM_CMD_CIPHEAD		(GSM_CMD_CIFSR +1)
#define GSM_CMD_CIPSTART	(GSM_CMD_CIPHEAD +1)
#define GSM_CMD_CIPSEND		(GSM_CMD_CIPSTART +1)
#define GSM_CMD_CIPCLOSE	(GSM_CMD_CIPSEND +1)
#define GSM_CMD_CIPSHUT		(GSM_CMD_CIPCLOSE +1)

/*The following table contains the strings to be sent to the GSM module for constructing a
full HTTP GET request.*/
const char* HTTPStrs[] = {
	"GET /Setts/",							//Performing a "Settings" request to server
	"GET /UIDSignal/",						//Performing a "Signal" request to server
	"?STim=",								//Includes the starting time of the request as a
											// GET parameter
	"&Stmp=",								//Starting of event timestamps list in the request
	" HTTP/1.1\r\nHost: ",					//Ending the request heading for the host
	"\r\nContent-Length: 0\r\n\r\n\x1a"		//Termination of the GET request
};
//Definitions of HTTP strings for easier handling and self documented code
#define HTTP_GETSET			(0)
#define HTTP_GETSIG			(HTTP_GETSET +1)
#define HTTP_ST				(HTTP_GETSIG +1)
#define HTTP_STMP			(HTTP_ST +1)
#define HTTP_HEAD			(HTTP_STMP +1)
#define HTTP_END			(HTTP_HEAD +1)

//Strings to be received from the GSM module while manipulating the HTTP request
const char InpData[] = "> ";				//String to expect for entering AT+CIPSEND data
const char IPDLen[] = "+IPD,";				//String to expect from server response
const char InSMSInd[] = "+CMTI: ";			//String to expect for a new SMS
const char ListSMSHead[]="+CMGL: ";			//String to expect after issuing a List SMS comm.

//Parameters returned from server and must be parsed by the associated functions
const strParamFunc ParamsArr[] = {			//Array of string parameters expected from server
  {"HTTP/1.1 ", SuccessComm},				//HTTP request was successful
  {"OK ", OKParse},							//Only set to skip the server's first OK
  {"FALSE ",FalseParse},					//Only set to skip the server's first False
  {"UpdTm:", UpdTmParse},					//Expresses the update time after <x> minutes
  {"NowTm:", NowTmParse},					//Expresses the current date/time
  {"Reset:", ResetParse},					//Expresses if the system should reset master
											// variables
  {"StmpOK:", SavedParse}					//Expresses the number of stamps saved by the
  											// server, successfully
};
//Definitions of parameters for easier handling and self documented code
#define OKPARAM			1					//Offset of OK parameter
#define FALSEPARAM		2					//Offset of False parameter
#define PARAMSNO		7					//Number of parameters in ParamsArr array

/*Parameters expected from SMS messages. Their values must be parsed by their associated
functions.*/
const strSMSParamFunc SMSParamArr[] = {		//Array with string parameters expected from SMS
  {"APN", APNParse},						//Setting APN parameter
  {"USERNAME", UserParse},					//Setting Username parameter
  {"PASSWORD", PassParse},					//Setting Password parameter
  {"HOST", HostParse},						//Setting Host parameter
  {"PORT", PortParse},						//Setting Server's port parameter
  {"UID", UIDParse},						//Setting UID parameter
  {"RESETUID", ResUIDParse},				//Command to reset UID to default (IMEI)
  {"RTCCAL", RTCCalParse},					//Calibrate RTC (value of AON_RTC_O_SUBSECINC)
  {"NODEUPDATE", NodeUpdParse}				//Setting of RF update time (RFUpdTm)
};
#define SMSPARAMSNO		9					//Number of SMS parameters in the array
//Lets calculate the maximum number of parameters to be parsed through the preprocessor
#if PARAMSNO > SMSPARAMSNO
#define MAXPARAMS		PARAMSNO
#else
#define MAXPARAMS		SMSPARAMSNO
#endif


/*********************************************************************************************
 * Variable definitions                                                                      *
 ********************************************************************************************/
#ifdef DEBUGFSM
strGSMStack GSMStack[DEBUGFSMSTACKSIZE];	//Array that holds the history of GSM FSM entries
int GSMStackStart;							//Keeps the starting offset of the Stack
int GSMStackLength;							//Keeps the stored values number in GSMStack
//unsigned long int TimeCounter;			//Keeps the number of Timer A0 interrupts
#endif

char GsmInBuffer[GSMINSIZE];				//Buffer that holds the response from GSM module

//Pins used for GSM Module
PIN_Handle GSMPinHandle;					//Pins are used through PIN drv. Handle of pins
PIN_State GSMPinState;						//State of handled ping used for GSM module

/*The system communicates to the module using the UART of the processor.
 * Here are the necessary variables */
UART_Params GSMUartPars;					//Parameters of the UART subsystem
UART_Handle GSMUartHandle;					//Handle of the UART object

/*The task needs a semaphore to be notified that a GPRS communication should be initiated */
Event_Handle GPRSEvt;						//The event handle to notify the GPRS task about
											// various events
//There also must be a semaphore for task sleeping
Semaphore_Handle GPRSSem;

///*The RTC of the system cannot be used by this code because it is in use by TI-RTOS. So, the
//usage of one shot clock objects is necessary.*/
//Clock_Handle ReGSMClk;
//
//Main task for manipulating the GSM module to achieve GPRS communication to server
Task_Struct GPRSStruct;						//Structure of the GPRS task
Char GPRSStack[GPRSSTACKSIZE];				//Stack of GPRS task

//State machine variables
long int UpdateTime;						//Holds the update interval since our last server
											// communication. It is in minutes
volatile uint32_t SystemStatus;				//Status flags for keeping the status of the FSM
uint32_t SMSStatus;							//Status flags for the reception of SMS
int16_t ParamsFound;						//Number of parameters found at HTTP response
uint16_t GSMSerialStatus;					//Status flags for the serial port of GSM
uint16_t GsmLength;							//Length of HTTP response in characters
uint16_t StampsSent;						//The number of timestamps sent in our last GPRS
											// communication
int16_t GsmLimitChrs;						//The limit of characters to be read as a
											// response
uint16_t SMSMemIndex;						//Current index of SMSMem array
uint16_t SMSMemLength;						//The length of used array
strSMSMem SMSMem[SMSMEMSIZE];				//Array to help reading SMS messages
uint8_t GsmFsmState;						//Keeps the state number
uint8_t	GsmRepeatCnt;						//Repeat counter of GSM AT commands' retries
uint8_t GsmRSSI;							//Fetched RSSI from GSM module
uint8_t GsmBER;								//Fetched BER from GSM module
uint8_t CurrNode;							//Current node served
uint8_t CurrNodeIdx;						//Current node's index in tables
uint8_t SndTm;								//Number of times we've sent signal request for
											// the master node
strParamSearch ParamSearch[MAXPARAMS];		//Array to help searching through server side
											// parameters (from GET request) or SMS text
char TimeBuffer[22];						//Buffer to hold current time string from server
char NextUpdateTime[22];					//Buffer to prepare the next update time string
volatile uint32_t ClkEvent;					//The event that is passed to the clock callback


/*********************************************************************************************
Debugging Helper Functions
*********************************************************************************************/
#ifdef DEBUGFSM
//Push state entering parameters to GSM Debugging Stack
void DEBUGENTER(int name) {
	//Lets check if the stack is full. Must keep the last events only
	int Stacki = GSMStackLength;
	if(Stacki == DEBUGFSMSTACKSIZE) {
		Stacki = GSMStackStart;
	}
	//Perform the parameters' pushing to stack
	GSMStack[Stacki].State = GsmFsmState;			//State needed to enter
	GSMStack[Stacki].RealState = name;				//State really entered
	GSMStack[Stacki].InTime = Clock_getTicks();		//Timestamp the state was entered
	GSMStack[Stacki].InSysStatus = SystemStatus;	//SystemStatus flags while entering
	GSMStack[Stacki].InSerStatus = GSMSerialStatus;	//GSMSerialStatus flags while entering
//	GSMStack[Stacki].InWakeTimes = GSMSWakeTimes;
}

//Push state exiting parameters to GSM Debugging Stack
void DEBUGEXIT(int path) {
	//Lets check if the stack is full and remove only the older entry
	int Stacki = GSMStackLength++;
	if(Stacki >= DEBUGFSMSTACKSIZE) {
		GSMStackLength = DEBUGFSMSTACKSIZE;
		Stacki = GSMStackStart++;
		if(GSMStackStart >= DEBUGFSMSTACKSIZE) {
			GSMStackStart = 0;
		}
	}
	//Perform the parameter's pushing
	GSMStack[Stacki].ExitPath = path;				//The exit path of the state
	GSMStack[Stacki].OutSysStatus = SystemStatus;	//SystemStatus flags while exiting
	GSMStack[Stacki].OutSerStatus = GSMSerialStatus;//GSMSerialStatus flags while exiting
//	GSMStack[Stacki].OutWakeTimes = GSMSWakeTimes;
}
#endif

/*********************************************************************************************
 * Main UART code                                                                            *
 ********************************************************************************************/
/*Callback function for tracking Status pin. When the STATUS signal is asserted, it triggers
this interrupt function and it informs the GSM FSM loop for the readiness of the module to
accept commands through UART. The FSM loop is informed by an event object.*/
void cbGSMStatus(PIN_Handle handle, PIN_Id pinId) {
	if(pinId == GSM_STATUS) {
		GSMSerialStatus |= GSMS_STATUS;		//Just came a rising edge of STATUS
		Event_post(GPRSEvt, EV_GPRSSEROK);	//Post an event to notify the readiness of GSM
		SystemStatus |= SS_GPRSSEROK;		//Also flag this in SystemStatus
	}
}


//********************************************************************************************
/*Clock callback function to trigger another GSM communication after UpdateTime has expired.
Of course, the signaling is performed by an Event object, only when the GSM module is off.
The input argument InArg is the event that will be sent to the GSM state machine.*/
void GSMRestart(UArg InArg) {
	Clock_stop(ReGSMClk);					//Stop the clock from running
	if((SystemStatus & SS_GSMON) == 0) {	//Is the GSM module powered?
		/*If yes, then there is nothing to do. If no, then we need to check if we must fire
		a starting event. There is the possibility of a triggering near the time the RF
		communication will take place. In that case we just perform a late triggering.*/
		if((RFStatusFlags & RFS_DELAYGPRS) != 0) {	//Is RF task in action?
			RFStatusFlags |= RFS_NEEDGPRS;			//Yes => Flag the necessity to trigger
													// the GPRS task after RF finishes
		}
		if((RFStatusFlags & RFS_DELAYGPRS) == 0) {	//No RF task in action?
			Event_post(GPRSEvt, ClkEvent);			//Trigger an event to the GPRS task. The
													// event is stored in ClkEvent variable
		}
	}
}


//********************************************************************************************
/* Initialize the task of GSM/GPRS handling */
void InitGPRSTask(void) {
	Task_Params GPRSPars;					//Parameters to control the task creation
	Event_Params GPRSEvtPars;				//Parameters to control the semaphore creation
	Semaphore_Params GPRSSemPars;			//Semaphore parameters

	//Lets initialize all the pins used for the GSM module
	GSMPinHandle = PIN_open(&GSMPinState, GSMPinTable);
	if(!GSMPinHandle) {
		System_abort("Error initializing board GSM pins\n");
	}
	PIN_registerIntCb(GSMPinHandle, cbGSMStatus);

	UART_init();							//Initialize the UART module
	UART_Params_init(&GSMUartPars);			//Initialize the UART parameters
	/*The following commented out lines are the default ones, or for testing purposes */
//	GSMUartPars.baudRate = 115200;
//	GSMUartPars.dataLength = UART_LEN_8;
//	GSMUartPars.parityType = UART_PAR_NONE;
//	GSMUartPars.stopBits = UART_STOP_ONE;
	GSMUartPars.readEcho = UART_ECHO_OFF;	//No need to echo back the input characters
	GSMUartPars.writeTimeout = 3 *1000000 / Clock_tickPeriod;	//Means 3 seconds
	GSMUartPars.readTimeout = 10 *1000000 /Clock_tickPeriod;	//Means 10 seconds
	/*There is the possibility of a low battery in a way that the GSM module cannot stay on,
	while the main processor can, as the latter can work at lower voltage levels than the GSM
	module. The power failure of the GSM module can easily happen during a UART_write command.
	If we leave the default value, BIOS_WAIT_FOREVER, as writeTimeout and there is a power
	failure at the module, then the code will wait forever for the module to receive data. To
	correct this behavior we set the writeTimeout value to 3 seconds. The only drawback is
	that we have to check for the exit status of the UART_write command and if there is a
	timeout we have to perform a normal power down (just in case the module is still powered
	on.*/

	//Event creation for notifications passed to GPRS communication task
	Event_Params_init(&GPRSEvtPars);
	GPRSEvt = Event_create(&GPRSEvtPars, NULL);

	//Semaphore creation for task sleeping hack
	Semaphore_Params_init(&GPRSSemPars);
	GPRSSem = Semaphore_create(0, &GPRSSemPars, NULL);

//	//Clock creation for repeating GSM communication
//	Clock_Params_init(&ClkParams);
//	ClkParams.startFlag = false;
//	ClkEvent = 0;
//	ReGSMClk = Clock_create(GSMRestart, DEFUPDTIME *1000000/Clock_tickPeriod, &ClkParams,
//    	NULL);
//
	//The main GPRS handling task creation
	Task_Params_init(&GPRSPars);
	GPRSPars.stackSize = GPRSSTACKSIZE;
	GPRSPars.priority = GPRSPRIORITY;
	GPRSPars.stack = &GPRSStack;
	Task_construct(&GPRSStruct, (Task_FuncPtr)GPRSTaskFxn, &GPRSPars, NULL);

	//Initialize necessary variables
	GsmFsmState = GFSM_OFF;					//The state of the module is Off
	SystemStatus = 0;						//No specific status right now...
	SMSStatus = 0;							//No SMS status defined...
	UpdateTime = DEFUPDTIME;				//Set the default update time
}


//********************************************************************************************
/*Force the state machine to power off the module. Gracefully exit from an unrecoverable
error.*/
void FSMEnterPowerOff(void) {
	PIN_setOutputValue(ledPinHandle, GSM_ERRORLED, 1);
	SystemStatus &= ~(SS_GPRSDELAY | SS_NEWDATA);//Clear the GPRS Delay
											// Timeout flag. The absence of SS_NEWDATA flags
											// an error state
	SystemStatus |= (SS_POWEROFF | SS_NOEXIT);
											//Have to follow another state at once (and
											// enable it through POWEROFF flag)
	GsmFsmState = GFSM_POWEROFF;			//Need to power off the module. Try later
}


//********************************************************************************************
/*Parses a literal value, whether it is in quotes (single or double) or not. It is used to
parse the values of APN, Username, Password, Host or UID from an SMS text. The input
parameters are InStr, points to the string to be parsed, OutStr points to the target buffer
the value will be stored, OutLength is the maximum length of the target buffer in characters,
including the terminating zero, Flag is the SMSStatus flag needed to be altered in case of
success or failure. The function returns the number of characters used in order to find the
parsed value.*/
int8_t ParseValue(char* InStr, char* OutStr, int OutLength, int Flag)
{
	int CharsNo = 0;						//Counts the number of characters used
	int Skipped = 0;						//Counts the number of characters skipped
	char Quote = *InStr;					//Get the quote character
	char TempChr;							//Current character manipulated

	SMSStatus &= ~Flag;						//Clear the target flag in SMS Status
	while(Quote == ' ') {					//Need to skip space characters (if they exist)
		Skipped++;							//Another one character skipped
		Quote = *(++InStr);					//Fetch next character
	}

	if((Quote != '"') && (Quote != '\'')) {	//Is it a kind of quote? (single or double)
		Quote = '\0';						//No => Quote is zeroed
	} else {								//Yes =>, Keep Quote kind
		InStr++;							//Skip this character, also
		Skipped++;							//Another one character skipped
	}
	while(CharsNo < OutLength) {			//Loop until the available value space is over
		TempChr = *InStr++;					//Get current character
		if(CharsNo == OutLength) {			//Final character? Need to terminate the value
			*OutStr = '\0';					//Terminate the string
			if(TempChr == '\0') {			//Did we copy the full string?
				SMSStatus |= Flag;			//Yes => Flag it as valid
			}
			break;							//Terminate the parsing
		}
		//If the character is a terminating one then exit. Flag the existence of a new APN
		if(((TempChr == Quote) && (Quote != '\0')) ||
				(((TempChr == ' ')  || (TempChr == ',') || (TempChr == '\0')) &&
					(Quote == '\0'))) {
//			CharsNo++;						//Also count in this terminating character
			*OutStr = '\0';					//Add the terminating character at the end of
			// target string
			SMSStatus |= Flag;				//Flag that username is set
			break;							//Normal exit
		}
		//If the character is not a valid one, exit. Do not accept value change
		if(((TempChr < '0') || ((TempChr > '9') && (TempChr < 'A')) ||
				((TempChr > 'Z') && (TempChr < 'a')) ||
				(TempChr > 'z')) && (TempChr != '-') && (TempChr != '.') &&
				(TempChr != '_')) {			//Something else than digits, letters, _, - or .?
			if(Flag == SMS_HOSTSET) {
				break;						//Host cannot accept any other character
			}
			if((Flag == SMS_APNSET) && (TempChr != '*')) {//APN also accepts *
				break;						//Not valid APN character? => exit
			}
			if(((Flag & (SMS_USERSET | SMS_UIDSET)) != 0) && (TempChr != '@')) {
				break;						//Username and UID accept also @
			}
			if((Flag == SMS_PASSSET) && (TempChr == ' ')) {
				break;						//Password accepts all characters but space
			}
		}
		//If the character is valid, store it in Flash buffer and count in the new character
		*OutStr++ = TempChr;				//Store this character and advance the target
											// pointer to the next character
		CharsNo++;							//Count in another one character
	}
	return (CharsNo + Skipped);
}


//********************************************************************************************
/*This function is called whenever "HTTP/1.1 " is received from server as an answer to a
"settings" or "signal" request. The value returned is the server's status. The only accepted
status id 200 (OK).  The function expects TempBuffer to contain the line read from server that
contains the server's parameter in question. The input parameter "offset" specifies the offset
in TempBuffer that contains the server's parameter value. The function returns the number of
characters used as the parameter's value.
*/
int8_t SuccessComm(int offset, uint8_t NodeAddr) {
	int CharsNo = 0;						//Counter of characters used
	int RetVal = 0;							//Return value is the HTTP status
	char InChar = GsmInBuffer[offset] - '0';	//Get the first character as a number
	//Going to convert the integer ascii value to a normal number
	while(InChar < 10) {					//Repeat loop until a non-digit character found
		RetVal = (RetVal *10) + InChar;
		CharsNo++;							//One more character used
		InChar = GsmInBuffer[offset + CharsNo] - '0';	//Read the next character as a number
	}
	if(RetVal == 200) {						//Request to server was OK
//	SMSTimer = 0;							//Clear SMS Time counter
//	RychoCounter = 0;						//Clear the number of optical events
//	__disable_interrupt();					//The next two commands need to be executed in a
//											// safe block, so disable interrupts
//	StampsCounter -= StampsSent;			//Count out the timestamps sent
//	StampsStart += StampsSent;				//Move the starting point of the buffer to the
//											// first position, just after the timestamps just
//											// sent.
//	if (StampsStart >= STAMPBUFSIZE)		//Cross the buffer borders?
//	  StampsStart -= STAMPBUFSIZE;			//Yes => Bring it in, again!
//	__enable_interrupt();					//Re-enable interrupts
		StampsSent = 0;						//Reset the stamps number sent. It will be updated
											// the next time a new packet of data is sent
//	GSMonCounter = 0;						//Clear the number of optical events while GSM on
	} else {								//If the request was not OK, then we must skip
											// searching for returned parameters
		SystemStatus |= SS_FOUNDALL;		//Flag that all parameters were found in order to
											// skip searching
	}
	return CharsNo;
}


//********************************************************************************************
/*This function is called whenever "OK " parameter is returned from server as an answer to
a "signal" request. The parameter has no value so we do not have to parse anything. It is used
to capture the line of correct parameters and only. The following character should be a '-'
and is the one that will be skipped as a delimiter character by the caller. The function
expects TempBuffer to contain the line read from server that contains the server's parameter
in question. The input "offset" specifies the offset in TempBuffer that contains the server's
parameter value. The function returns the number of characters used as the parameter's value.
Since there is no real parameter value, the function returns 0. OK and False are mutual
exclusive, so OK sets status of False as Found also
*/
int8_t OKParse(int offset, uint8_t NodeAddr) {
	ParamSearch[FALSEPARAM].offset = 0;
	ParamSearch[FALSEPARAM].status = PS_FOUND;
	ParamsFound++;
	if((NodeAddr == DEF_MASTER_ADDRESS) ||
		(NodeAddr == DEF_BROADCAST_ADDRESS)) {
		SndTm++;
	} else {
		NodesParsArr[CurrNodeIdx].SndTm++;
	}
	return 0;
}


//********************************************************************************************
/*This function is called whenever "False " parameter is returned from server as an answer to
a "signal" request. The parameter has no value so we do not have to parse anything. It is used
to capture the line of correct parameters and only. The following character should be a '-'
and is the one that will be skipped as a delimiter character by the caller. The function
expects TempBuffer to contain the line read from server that contains the server's parameter
in question. The input "offset" specifies the offset in TempBuffer that contains the server's
parameter value. The function returns the number of characters used as the parameter's value.
Since there is no real parameter value, the function returns 0. OK and False are mutual
exclusive, so False sets status of OK as Found also
*/
int8_t FalseParse(int offset, uint8_t NodeAddr) {
	ParamSearch[OKPARAM].offset = 0;
	ParamSearch[OKPARAM].status = PS_FOUND;
	ParamsFound++;
	return 0;
}


//********************************************************************************************
/*This function is called whenever "UpdTm:" parameter is returned from server as an answer to
a "settings" or "signal" request. The value of the parameter returned expresses the update
interval in minutes, set by the user of the system. The function expects TempBuffer to contain
the line read from server that contains the server's parameter in question. The input "offset"
specifies the offset in TempBuffer that contains the server's parameter value. The function
returns the number of characters used as the parameter's value
*/
int8_t UpdTmParse(int offset, uint8_t NodeAddr) {
	int CharsNo = 0;						//Counter of characters used
	long int NewUpdTm = 0;					//Update time from HTTP response

//	UpdateTime = 0;							//Reset UpdateTime
	char InChar = GsmInBuffer[offset] - '0';//Get the first character as a number
	//Going to convert the integer ascii value to a normal number
	while(InChar < 10) {					//Repeat loop until a non-digit character found
		NewUpdTm = (NewUpdTm *10) + InChar;
		CharsNo++;							//One more character used
		//Now read the next character an convert it from ASCII to decimal number
		InChar = GsmInBuffer[offset + CharsNo] - '0';
	}
	if(NewUpdTm == 0) {					//Is the time equal to 0 minutes?
		NewUpdTm = DEFUPDTIME;				//Yes => then use the default time interval
	}
	//There cannot be a longer update interval than 5 days, so if it is greater, truncate it
	if(NewUpdTm > 7200) NewUpdTm = 7200;

	//Finally, the UpdateTime is set only if the response is for the master node, otherwise it
	// is ignored
	if(NodeAddr == DEF_MASTER_ADDRESS) {
		UpdateTime = NewUpdTm;
	}
	return CharsNo;							//Return the number of characters used
}


//********************************************************************************************
/*This function is called whenever "NowTm:" parameter is returned from server as an answer to
a "settings" or "signal" request. The value of the parameter returned expresses the current
date/time. The function expects TempBuffer to contain the line read from server that contains
the server's parameter in question. The input "offset" specifies the offset in TempBuffer that
contains the server's parameter value. The function returns the number of characters used as
the parameter's value (in this case, nineteen characters construct the whole date/time in the
form "YYYY/MM/DD hh:mm:ss")
*/
int8_t NowTmParse(int offset, uint8_t NodeAddr) {
	strncpy(TimeBuffer, GsmInBuffer + offset, 19);//Copy 19 characters just after "NowTm:' to
											//TimeBuffer. They contain the current time from
											// server.
	TimeBuffer[19]=0x0;						//Add the terminating character
	if(NodeAddr == DEF_MASTER_ADDRESS) {	//The RTC is updated only if the response is for
											// the master node. otherwise it is ignored
		RTCReset(TimeBuffer);				//Initialize RTC registers to the current time
											// in TimeBuffer
	}
	return 19;
}


//********************************************************************************************
/*This function is called whenever "Reset:" parameter is returned from server as an answer to
a "settings" or "signal" request. If the value of the parameter returned is '1' then all the
main variables are reset. The function expects GsmInBuffer to contain the line read from
server that contains the parameter in question. The input "offset" specifies the offset in
GsmInBuffer that contains the server's parameter value. The function returns the number of
characters used as the parameter's value (in this case, only one character is the value)
eliaschr@NOTE: Pitfall Master Trap doesn't make use of this parameter, cause it does not keep
records of insects. It is just a bridge between the wireless nodes and the internet. The real
use of this parameter is while communicating to server for getting parameters for a wireless
trap.
*/
int8_t ResetParse(int offset, uint8_t NodeAddr) {
	if (GsmInBuffer[offset] == '1') {		//Is the value of the found parameter equal to 1?
											//Yes => Reset the counters
		if(NodeAddr > DEF_MASTER_ADDRESS) {	//Normal node or master one?
			//Set the flag to inform the slave that needs to be reset at the next RF update
			NodesParsArr[NodeAddr -2].flags |= NPF_GSMRESET;
		} else if (NodeAddr == DEF_MASTER_ADDRESS) {
			//In case the master is informed to reset, it dispatches this request to ALL its
			// slave nodes.
			for(NodeAddr = 0; NodeAddr < NodesCnt; NodeAddr++) {
				NodesParsArr[NodeAddr].flags |= NPF_GSMRESET;
			}
		}
	}
	return 1;								//Only one character was used
}


//********************************************************************************************
/*This function is called whenever "NumOfSavedStamps:" parameter is returned from server as an
answer to a "signal" request. The value of the parameter returned is used to understand is
timestamps were sent in a proper format or not. The function expects GsmInBuffer to contain
the line read from server that contains the server's parameter in question. The input "offset"
specifies the offset in GsmInBuffer that contains the server's parameter value. The function
returns the number of characters used as the parameter's value.
eliaschr@NOTE: Actually this value is not used anywhere. It could really be a feedback from
the server, but as long as it does not describe what is wrong it cannot be really used. Need
to improve the protocol with the server to be really valuable
*/
int8_t SavedParse(int offset, uint8_t NodeAddr) {
	int i;									//Helper counter for loops
	int CharsNo = 0;						//Counter of characters used
	int TempStamps = 0;						//Reset TempStamps that will hold the value

	char InChar = GsmInBuffer[offset] - '0';//Get the first character as a number
	//Going to convert the integer ascii value to a normal number
	while(InChar < 10) {					//Repeat loop until a non-digit character found
		TempStamps = (TempStamps *10) + InChar;
		CharsNo++;							//One more character used
		InChar = GsmInBuffer[offset + CharsNo] - '0';	//Read the next character as a number
	}
//	StampsSent = TempStamps;				//Get the number of stamps the server reported as
											// stored
	if(TempStamps == 0) {					//If no stamps stored..
		return CharsNo;						//... => Nothing more to do
	}
	//Need to reset the first StampsSent stamps of this node from the events array
	TempStamps = StampsSent;
	for(i = 0; ((i < MAXNODEEVENTS) && (TempStamps > 0)); i++) {
		if(EvArray[i].NodeShortAddr == NodeAddr) {
			EvArray[i].NodeShortAddr = 0;
			TempStamps--;
		}
	}
	NodesParsArr[NodeAddr -2].StampsNo -= StampsSent;
	NodesEvCnt -= StampsSent;
	return CharsNo;
}


//********************************************************************************************
/*Fetches APN string from SMS. The input parameter offset contains the offset in TempBuffer
that APN lies. The return value is the number of characters processed. The APN can be in
quotes and the maximum number of characters is APNLENGTH -1. That is because the last one must
be the terminating '\0'. The fetched APN is written in FlashBuffer variable. This prepares the
segments to be written in Flash. The APN is a name consisting of letters (capital or not),
numbers, underscores, dashes and dots.*/
int8_t APNParse(int offset) {
	int8_t i;
	int16_t i16;
	for(i16 = APNOFFSET; i16 < (APNOFFSET +APNLENGTH); i16++) {
		FlRAM.ui8Buf[i16] = 0xFF;
	}
	//Lets parse the APN string
	i = ParseValue(&GsmInBuffer[offset], &FlRAM.cBuf[APNOFFSET], APNLENGTH, SMS_APNSET);
	if(strcmp(APN, &FlRAM.cBuf[APNOFFSET]) == 0) {//Is it the same as before?
		SMSStatus &= ~SMS_APNSET;			//Yes => Do not save it in Flash
	}
	return i;								//Return the number of characters used
}


//********************************************************************************************
/*Fetches Username string from SMS. The input parameter offset contains the offset in
TempBuffer that username value lies. The return value is the number of characters processed.
The username can be in quotes and the maximum number of characters is USERLENGTH -1. That is
because the last one must be the terminating '\0'. The fetched username is written in
FlashBuffer variable. This prepares the segments to be written in Flash. The username is a
name consisting of letters (capital or not), numbers, at symblols (@), underscores, dashes and
dots.*/
int8_t UserParse(int offset) {
	int8_t i;
	int16_t i16;
	for(i16 = USEROFFSET; i16 < (USEROFFSET +USERLENGTH); i16++) {
		FlRAM.ui8Buf[i16] = 0xFF;
	}
	//Lets parse the username string
	i = ParseValue(&GsmInBuffer[offset], &FlRAM.cBuf[USEROFFSET], USERLENGTH, SMS_USERSET);
	if(strcmp(Username, &FlRAM.cBuf[USEROFFSET]) == 0) {//Is it the same as before?
		SMSStatus &= ~SMS_USERSET;				//Yes => Do not save it in flash memory
	}
	return i;									//Return the number of characters used
}


//********************************************************************************************
/*Fetches Password string from SMS. The input parameter offset contains the offset in
TempBuffer that password value lies. The return value is the number of characters processed.
The password can be in quotes and the maximum number of characters is PASSLENGTH -1. That is
because the last one must be the terminating '\0'. The fetched password is written in
FlashBuffer variable. This prepares the segments to be written in Flash. The password is a
value consisting of letters (capital or not), numbers, and symbols. The only unaccepted
character is space.*/
int8_t PassParse(int offset) {
	int8_t i;
	int16_t i16;
	for(i16 = PASSOFFSET; i16 < (PASSOFFSET +PASSLENGTH); i16++) {
		FlRAM.ui8Buf[i16] = 0xFF;
	}
	//Lets parse the password string
	i = ParseValue(&GsmInBuffer[offset], &FlRAM.cBuf[PASSOFFSET], PASSLENGTH, SMS_PASSSET);
	if(strcmp(Password, &FlRAM.cBuf[PASSOFFSET]) == 0) {//Is it the same as before?
		SMSStatus &= ~SMS_PASSSET;				//Yes => Do not save it in flash memory
	}
	return i;									//Return the number of characters used
}


//********************************************************************************************
/*Fetches Host string from SMS. The input parameter offset contains the offset in TempBuffer
that Host lies. The return value is the number of characters processed. The Host can be in
quotes and the maximum number of characters is HOSTLENGTH -1. That is because the last one
must be the terminating '\0'. The fetched Host is written in FlashBuffer variable. This
prepares the segments to be written in Flash. The Host is a name consisting of letters
(capital or not), numbers, underscores, dashes and dots.*/
int8_t HostParse(int offset) {
	int8_t i;
	int16_t i16;
	for(i16 = HOSTOFFSET; i16 < (HOSTOFFSET +HOSTLENGTH); i16++) {
		FlRAM.ui8Buf[i16] = 0xFF;
	}
	//Lets parse the host string
	i = ParseValue(&GsmInBuffer[offset], &FlRAM.cBuf[HOSTOFFSET],
		HOSTLENGTH, SMS_HOSTSET);
	if(strcmp(Host, &FlRAM.cBuf[HOSTOFFSET]) == 0) {//Is it the same as before?
		SMSStatus &= ~SMS_HOSTSET;				//Yes => Do not save it in flash memory
	}
	return i;									//Return the number of characters used
}


//********************************************************************************************
/*Fetches UID string from SMS. The input parameter offset contains the offset in TempBuffer
that UID lies. The return value is the number of characters processed. The UID can be in
quotes and the maximum number of characters is UIDLENGTH -1. That is because the last one must
be the terminating '\0'. The fetched UID is written in FlashBuffer variable. This prepares the
segments to be written in Flash. The UID is a name consisting of letters (capital or not),
numbers, underscores, dashes, dots and 'at' (@) symbols.*/
int8_t UIDParse(int offset) {
	int8_t i;
	int16_t i16;
	for(i16 = UIDOFFSET; i16 < (UIDOFFSET +UIDLENGTH); i16++) {
		FlRAM.ui8Buf[i16] = 0xFF;
	}
	//Lets parse the UID string
	i = ParseValue(&GsmInBuffer[offset], &FlRAM.cBuf[UIDOFFSET],
		UIDLENGTH, SMS_UIDSET);
	if(strcmp(UID, &FlRAM.cBuf[UIDOFFSET]) == 0) {//Is it the same as before?
		SMSStatus &= ~SMS_UIDSET;				//Yes => Do not save it in flash memory
	}
	return i;									//Return the number of characters used
}


//********************************************************************************************
/*Fetches Port string from SMS. The input parameter offset contains the offset in TempBuffer
that port value lies. The return value is the number of characters processed. The port can be
in quotes (single or double). The fetched port value is written in FlashBuffer variable. This
prepares the segments to be written in Flash. The port is an integer value, so it can be up to
65535. It can be specified using C style notation, using prefix '0x' for hexadecimal, '0b' for
binary or no prefix at all for decimal numbering system. It can also be specified in assembly
notation, meaning that the postfix 'h', or 'b' specify hexadecimal or binary notation
respectively. No postfix means decimal numbering. Using both prefix and postfix does not harm
the parsing process, as long as both prefix and postfix specify the same numbering system.*/
int8_t PortParse(int offset) {
	int8_t CharsNo;							//Number of characters used for parsing
	int CurrPort = atoi(Port);				//Get current port
	uint16_t Result = CurrPort;				//The result of number parsing, as default value
	char TempNum[PORTLENGTH];				//String of PORTLENGTH bytes. It can hold an
											// integer

	for(CharsNo = 0; CharsNo < PORTLENGTH; CharsNo++) {
		TempNum[CharsNo] = 0xFF;
		FlRAM.ui8Buf[PORTOFFSET +CharsNo] = 0xFF;
	}
	SMSStatus &= ~SMS_PORTSET;				//Port value is not set
	CharsNo = ParseNumber(&GsmInBuffer[offset], &Result);//Parse the number
	if(CharsNo > 0) {						//Positive number of characters => Parsed value
		if(Result != CurrPort) {			//Is the value different than what it was before?
			/*Need to call Int2Ascii to convert this number to ASCII again in FlashBuffer*/
			TempNum[5] = '\0';				//Terminating character of string number
			CurrPort = Int2Ascii(Result, TempNum, 5);//Use CurrPort again as the number of
											//characters used for the number.
			/*Now, TempNum contains the integer number in ASCII format, but it is right
			aligned. We use it to bring it left aligned in FlashBuffer.*/
			strcpy(&FlRAM.cBuf[PORTOFFSET], &TempNum[5 -CurrPort]);
			if(strcmp(Port, &FlRAM.cBuf[PORTOFFSET]) != 0) {//Is it different than before?
				SMSStatus |= SMS_PORTSET;	//Flag the need to store the new value
			}
		}
	} else {								//Error on parsing =>
		CharsNo = -CharsNo;					//Just make the number positive to skip them
	}
	return CharsNo;							//Return the number of characters used
}


//********************************************************************************************
/*This function is a placeholder for reseting UID. It is not really implemented as using the
UID command can set whatever UID needed.*/
int8_t ResUIDParse(int offset) {
	return 0;
}


//********************************************************************************************
/*Low frequency crystal is not 32768 Hz exactly. There is a mechanism in the microcontroller
that can calibrate the RTC clock to work with the exact frequency of the connected crystal.
The calibration value is set using RTCCAL parameter in an SMS.*/
int8_t RTCCalParse(int offset) {
	int8_t CharsNo;							//Number of characters used for parsing
	uint32_t Result;						//The result of number parsing, as default value
	uint32_t DefVal;						//The default calibration value

	for(CharsNo = 0; CharsNo < RTCLENGTH; CharsNo++) {
		FlRAM.ui8Buf[RTCOFFSET +CharsNo] = 0xFF;
	}
	//Set the default calibration value from RTC
	DefVal = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSECINC);
	SMSStatus &= ~SMS_CALSET;				//Calibration value is not set
	CharsNo = ParseNumber32(&GsmInBuffer[offset], &Result);//Parse the number
	if(CharsNo > 0) {						//Positive number of characters => Parsed value
		if(Result != DefVal) {				//Is the value different than what it was before?
			/*Need to store the new value in Flash Buffer and the RTC register*/
			HWREG(AON_RTC_BASE + AON_RTC_O_SUBSECINC) = Result;
			FlRAM.ui32Buf[RTCOFFSET/4] = Result;
			SMSStatus |= SMS_CALSET;		//Flag the need to store the new value
		}
	} else {								//Error on parsing =>
		CharsNo = -CharsNo;					//Just make the number positive to skip them
	}
	return CharsNo;							//Return the number of characters used
}


//********************************************************************************************
/*Reads the SMS parameter of NODEUPDATE command and sets the interval in number of minutes the
RF nodes will connect to the master and exchange data.*/
int8_t NodeUpdParse(int offset) {
	int8_t CharsNo;							//Number of characters used for parsing
	uint16_t Result;						//The result of number parsing, as default value

	for(CharsNo = 0; CharsNo < NODTMLENGTH; CharsNo++) {
		FlRAM.ui8Buf[NODTMOFFSET +CharsNo] = 0xFF;
	}
	SMSStatus &= ~SMS_NODTMSET;				//Node Update Time value is not set
	CharsNo = ParseNumber(&GsmInBuffer[offset], &Result);//Parse the number
	if(CharsNo > 0) {						//Positive number of characters => Parsed value
		if(Result != *NodTm) {				//Is the value different than what it was before?
			/*Need to store the new value in Flash Buffer.*/
			FlRAM.ui16Buf[NODTMOFFSET/2] = Result;
			SMSStatus |= SMS_NODTMSET;		//Flag the need to store the new value
		}
	} else {								//Error on parsing =>
		CharsNo = -CharsNo;					//Just make the number positive to skip them
	}
	return CharsNo;							//Return the number of characters used
}


//==========================================
/* Main Task function */
Void GPRSTaskFxn(UArg arg0, UArg arg1) {
	UARTCC26XX_Object *UartParams;			//Need to tweak UART parameters...
	UInt ev;								//Event received
	DHCPEntry* NodePtr;						//Points to the MAC address of a registered node
	int32_t stTime;							//Starting time in clock ticks for timing events
	int32_t nowTime;						//Current time in clock ticks
	int32_t endTime;						//Ending number of clock ticks for the event
	uint32_t NowTime;						//Now timestamp from the RTC clock
	int CharCnt;							//Character counter for parameter searching
	int NFCount;							//Parameters not found counter
	int SearchState;						//The state of the search engine FSM
	int CurrParam;							//Current parameter to be checked
	int ChargePercent;						//Charging level of battery
	int MaxCounter;							//Maximum number of timestamps to be sent
	int StampsCounter;						//Number of stamps that appear in the list from
											// current node
	int RychoMasterCounter;					//The master counter of the current node
	int GSMonCounter;						//Current counter with GSM on (not really used)
	int GSMonMCounter;						//Current Master counter with GSM on (not really
											// used)
	int16_t SendTimes;						//Number of times we've sent a signal request for
											// the current node
	int16_t InChars;						//Number of characters received or handled
	int16_t TestPar;						//Helper variable for testing of UART data
	int16_t TestVal;						//Value that will hold a 16 bit parsed number
	uint16_t TestUVal;						//Test value unsigned
	uint16_t Vbatt;							//Battery voltage usage
	int16_t tmpLen;							//Helper variable for parsing PDU data
	uint8_t NodeRSSI;						//RSSI value of current node
	uint8_t NodeBER;						//BER value of current node
	char InChr;								//Character to be manipulated

	while(1) {
		//Wait for the event that signals the need of initiating a GPRS communication.
		ev = Event_pend(GPRSEvt, Event_Id_NONE, EV_GPRSALL, BIOS_WAIT_FOREVER);
//		Clock_stop(ReGSMClk);				//Just in case the triggering was not by the clock
		ClkEvent = ev;						//Store the event that triggered the FSM.
		RFStatusFlags &= ~(RFS_DELAYGPRS | RFS_NEEDGPRS);
		/*In case of error, the FSM will be triggered by the same event when ReGSMClk time
		expires. That is why we need to store the event to the clock argument.*/
		//Reset the New Data and Delay Timeout flags
		SystemStatus &= ~(EV_GPRSALL | EV_GPRSSEROK);
		SystemStatus |= ev;
		SystemStatus &= ~(SS_NEWDATA | SS_GPRSDELAY);
		//Lets start from the first node. This cannot be the broadcast short address
		CurrNode = 1;
		CurrNodeIdx = 0;

		//GSM Finite State Machine Loop starts here
        do {
        	SystemStatus &= ~(SS_NOEXIT);	//If there is no other reason, the FSM will exit

        	/*The following code constructs the whole Finite State Machine that organizes the
        	communication to the GSM module.*/
			switch(GsmFsmState) {
			case GFSM_OFF:					//GSM module is still not powered. So...
				DEBUGENTER(GFSM_OFF);
				SystemStatus |= SS_GSMON;	//Going to use GSM module
				//Lets initialize the UART object and subsystem and keep its parameter array
				GSMUartHandle = UART_open(0, &GSMUartPars);
				if(!GSMUartHandle) {
					System_abort("Error opening the GSM UART\n");
				}
				UartParams = (UARTCC26XX_Object*) GSMUartHandle->object;
				/*We prefer partial return. Whenever the FIFO times out, meaning that there
				is more time than 4 characters length that there is no incoming data, while
				the FIFO is not empty, the UART module asserts a Timeout Interrupt. Also, the
				maximum timeout interval is set to 1 second for data reading from GSM module*/
				UartParams->readTimeout = 1000000/Clock_tickPeriod;
				UART_control(GSMUartHandle, UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE, NULL);
				/*eliaschr@NOTE: In here the leds are used to signal various things about the
				communication. Blue led signal the application of power to the GSM module.
				Red led signals an error. Green led signals the starting of a GPRS communi-
				cation. In the end application there can be leds, but only a switch could
				activate them for a short time. In that way we preserve the battery, as those
				leds stay off, until needed by the user to be informed of the communication
				status.*/
				PIN_setOutputValue(GSMPinHandle, GSM_VCC, 1);	//Apply power to GSM module
				PIN_setOutputValue(ledPinHandle, GSM_ACTIVELED, 1);	//Signal Blue led
				PIN_clrPendInterrupt(GSMPinHandle, GSM_STATUS);
				PIN_setInterrupt(GSMPinHandle, GSM_STATUS | PIN_IRQ_POSEDGE);
				GsmFsmState = GFSM_MODULEPWR;//Now the GSM module is powered by the battery
				SMSStatus = 0;				//Reset the SMS status
				//Must wait 50 mSec for power voltage to settle
//				Task_sleep(50000/Clock_tickPeriod);
				Semaphore_pend(GPRSSem, 50000 / Clock_tickPeriod);
				//After the Task_sleep, the next state is going to be entered at once
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_MODULEPWR:			//Need to Power On the module. So...
				DEBUGENTER(GFSM_MODULEPWR);
				//Push the power button of the GSM module
				PIN_setOutputValue(GSMPinHandle, GSM_PWRKEY, 1);
				GsmFsmState = GFSM_PBPUSHED;//Current state is "Power Button Pushed"
				//The power key must be pressed for at least one second so wait
//				Task_sleep(1000000/Clock_tickPeriod);
				Semaphore_pend(GPRSSem, 1000000 / Clock_tickPeriod);
				//After the Task_sleep, the next state is going to be entered at once
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_PBPUSHED:				//Need to release the power button, so...
				DEBUGENTER(GFSM_PBPUSHED);
				/*Normally, this state can be triggered by two different events; Delay
				Timeout that means we have to lower the power button of the GSM module, or
				new data came that means the init sequence was met earlier than expected. In
				case of using UART without callback, the Init sequence from the GSM module
				must have been discarded silently by the UART, since no UART_read function is
				called and RX is not yet enabled, to conserve power.*/
				//Time to release the power button.GSM module is on
				PIN_setOutputValue(GSMPinHandle, GSM_PWRKEY, 0);
				/*Though there are some flags in SystemStatus that are deprecated, they are
				being used in order to have a better debugging process.*/
//				SystemStatus &= ~SS_RESPONSE;//Going to use command mode
				GsmFsmState = GFSM_SEROK;	//Going to check serial port status
				/*Wait for the GSM module to become ready, for at most 7 seconds. The
				signaling of the UART readiness state will come from STATUS pin interrupt.*/
				ev = Event_pend(GPRSEvt, Event_Id_NONE, EV_GPRSSEROK,
					7000000/Clock_tickPeriod);
				//If we haven't received the Init sequence within this time then something is
				// wrong
				DEBUGEXIT(0);
				//After the Event_pend the next state is entered at once. No break needed

			//--------------------------------------
			case GFSM_SEROK:				//Waits until the serial port is ready to accept
				DEBUGENTER(GFSM_SEROK);
				if((ev & EV_GPRSSEROK) != 0) {//Serial port is ready now
					//Command mode, serial port is ready
					//Clear unnecessary flags, going to enter command mode at next state
//					SystemStatus &= ~(SS_GPRSDELAY | SS_RESPONSE);
					GsmFsmState = GFSM_SETAT;//Going to issue a dummy AT command to help auto
											// baud rate synchronize with us
					//Again, the following state is entered at once, so no need for break
					DEBUGEXIT(0);
				} else if(ev == Event_Id_NONE) {//Timed out? Need to exit, something is wrong
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;
				}

			//--------------------------------------
			case GFSM_SETAT:				//Issue a dummy AT to sync auto baud rate
				DEBUGENTER(GFSM_SETAT);
				//Command mode
//				SystemStatus &= ~(SS_NEWDATA | SS_SECLINE);//Reset this flag, expect
											// first line of response (the GSM module's echo)
				SystemStatus |= (SS_RESPONSE | SS_NOEXIT);//Going to enter Response mode
				//Send a dummy AT command. Use time interval of 1 second to expect an answer
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_AT], 4);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				/*Lets perform a blocking Read. Expecting the echo back: "AT\r\n", total of 4
				characters.*/
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 4);
				//First line is the echo of AT command. Nothing to do, really, just
				// receive the second one
				if(InChars == 0) {			//If the number of characters read is not 4, then
					//there was a timeout. No characters received during a 1 seconds frame.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
//				SystemStatus |= SS_SECLINE;	//Next time we will receive the second line
				//Lets read the second line of the GSM response. It should be a "\r\nOK\r\n"
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 6);
				//Second line of response is the OK litteral. Cannot be anything else
				if(InChars == 0) {			//If the number of characters read is not 6, then
					// there was a timeout. No characters received during a 1 seconds frame.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(254);
					break;					//Exit the state
				}
//				SystemStatus &= ~(SS_SECLINE | SS_RESPONSE);//Enter next state at command mode
				GsmFsmState = GFSM_SETECHO;		//Will stop echoing issued commands
				DEBUGEXIT(0);
				//NEWDATA is still active, so SETECHO is going to be entered at once

			//--------------------------------------
			case GFSM_SETECHO:				//Going to issue an Echo Off command to GSM module
				DEBUGENTER(GFSM_SETECHO);
				//Command mode
//				SystemStatus &= ~(SS_NEWDATA | SS_SECLINE);//Reset this flag, expect first
				// line of response
//				SystemStatus |= SS_RESPONSE;//Going to enter Response mode
				UartParams->readTimeout = 1000000/Clock_tickPeriod;
				//Send 'ATE0' to module to disable echo
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_ATE0], 6);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
//				SystemStatus |= (SS_SECLINE | SS_RESPONSE);	//Next time we are going to handle
				// the first line of the response
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 8);	//Expecting "\r\nATE0\r\n"
				if(InChars == 0) {			//If the number of characters read is not 4, then
					// there was a timeout. No characters received during a 10 seconds frame.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				//Response mode, normal data line received, expecting the ATE0 echo
//				SystemStatus &= ~(SS_NEWDATA | SS_SECLINE);//Next time wait for the 2nd line
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 6);	//Expecting "\r\nOK\r\n"
				if(InChars == 0) {			//If the number of characters read is not 4, then
					// there was a timeout. No characters received during a 1 second frame.
					//Timeout (or possible false answer by the GSM module) => Error. Need
					// to power off
					FSMEnterPowerOff();
					DEBUGEXIT(254);
					break;					//Exit the state
				}
//				SystemStatus &= ~SS_RESPONSE;//Going to enter command mode of the next state
				GsmFsmState = GFSM_FLOWCTL;
				/*The rest of the code that runs in this state should not break. In this way
				the system enters GFSM_FLOWCTL at once.*/
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_FLOWCTL:				//Going to setup hardware flow control for UART
				DEBUGENTER(GFSM_SETECHO);
				//Command mode
//				SystemStatus &= ~(SS_NEWDATA | SS_SECLINE);//Reset this flag, expect first
				// line of response
				SystemStatus |= SS_RESPONSE;		//Going to enter Response mode
				//Send 'AT+IFC' to module to enable hardware control, both ways
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_IFC], 12);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
//				SystemStatus |= (SS_SECLINE | SS_RESPONSE);	//Next time we are going to handle
				// the first line of the response
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 6);	//Expecting "\r\nOK\r\n"
				if(InChars == 0) {			//If the number of characters read is not 6, then
					// there was a timeout. No characters received during a 10 seconds frame.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				//Response mode, normal data line received, expecting the ATE0 echo
//				SystemStatus &= ~(SS_NEWDATA | SS_RESPONSE | SS_SECLINE);//Next time wait for
				// the second line
				GsmRepeatCnt = 0;			//Clear the repetition counter
				/*It is time to find out if we need to learn our UID. The UID is read from
				IMEI of the GSM module. The first request being used is the 'settings'. For
				this request we must check if there is any UID in non volatile memory. If not
				then we ask the module for its IMEI and store the resulting number in UID.
				Signal requests follow, so there is no need to check for a UID. It is already
				stored in non-volatile memory from a previous Settings request.*/

				if((SystemStatus & SS_GSMTYPEREQ) != 0) {//Signal request? =>
					// IMEI is in Flash memory so proceed to setup a GPRS communication
					GsmFsmState = GFSM_COMMOK;
				} else {					//Settings request? =>
					if(UID[0] == 0xFF) {			//Do we have an UID? No =>
						//This should happen only once; The very first time the task runs
						GsmFsmState = GFSM_ASKIMEI;	//Proceed to IMEI fetching from module
						SystemStatus |= SS_NOEXIT;	//Transfer there at once
						DEBUGEXIT(1);
						break;						//Exit this state
					} else {						//Yes =>
						GsmFsmState = GFSM_COMMOK;	//Skip fetching IMEI from GSM Module
					}
				}
				/*The rest of the code that runs in this state should not break. In this way
				the system enters GFSM_COMMOK at once.*/
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_COMMOK:				//Going to connect to provider
				DEBUGENTER(GFSM_COMMOK);
				if((SystemStatus & SS_GPRSDELAY) != 0) {
//					SystemStatus &= ~SS_RESPONSE;		//Going to use command mode
					/*Since we do not clear the GPRSDELAY flag, in case of more retries
					possible, we enter the command mode delay that resends the command.*/
					if(++GsmRepeatCnt == CREG_MAXREPEAT) {//Run out of retries? Yes =>
						FSMEnterPowerOff();
						DEBUGEXIT(254);
						break;				//Exit this state to enter the following one
					}
				}
				/*If there are more retries then we reach this point of code. Command mode
				follows to re-send the registration status command*/
				//Command mode. Going to issue AT+CREG? command
				/*Clear the GPRSDELAY flag, next time we will be in Response mode for the
				first line*/
//				SystemStatus &= ~(SS_GPRSDELAY | SS_SECLINE);
				SystemStatus &= ~SS_GPRSDELAY;
//				SystemStatus |= SS_RESPONSE;
				UartParams->readTimeout = 2000000/Clock_tickPeriod;
				//Issue command to check registration status and fetch the response
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CREG], 10);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 30);
				/*If a timeout occurs then we have to re-send the command, so raise GPRSDELAY
				flag and re-enter the same state*/
				if(InChars == 0) {
					SystemStatus |= (SS_GPRSDELAY | SS_NOEXIT);
					DEBUGEXIT(128);
					break;
				}
				//Response mode of state
				/*Possible returns from "AT+CREG?" are "+CREG: <n>,<stat>[,<lac>,<ci>]" on
				success, or "+CME ERROR: <err>" on failure. Since we don't care about the kind
				of error in second case, we only need to check the third character to find out
				if we have an error in ME or not. Keep in mind that sometimes the module sends
				rubbish and the fact that the first two characters of the response are "\r\n".
				One more thing is that the following "\r\nOK\r\n" is read at the same
				UART_read command.*/
				if(GsmInBuffer[4] == 'R') {
					//Correct answer, fetch registration status code, <status> as an integer
					TestPar = GsmInBuffer[11] - '0';
					/*Lets find out if there is the whole command response in the buffer. Find
					the first occurrence of CRLF, include both leading and trailing CRLFs and
					check if there are more characters to contain "OK". If not, we must read
					the rest!*/
					TestVal = FindCRLF(&GsmInBuffer[2], InChars -2) +4;
					if(TestVal > (InChars -6)) {
						InChars = UART_read(GSMUartHandle, &GsmInBuffer[InChars], 30);
					}
				} else if (GsmInBuffer[4] == 'M') {
					//Received +CME ERROR
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Go there at once (Do not check for a retry)
				} else {
					//Received rubbish characters, so cannot conclude
					TestPar = -1;			//No correct parameter number
				}
				/*TestPar now contains the <stat> parameter of the command's response (by
				default, since we haven't changed it), the <n> parameter is 0). The <stat>
				value can be 0 to 5. 1 means "Registered to home network", 5 means
				"Registered to roaming network" and all the other values mean that it is not
				registered. Remember, there can be rubbish. In that case TestPar is -1.*/
				if((TestPar == 1) || (TestPar == 5)) {
					//We are registered to cellular telephony network
					GsmRepeatCnt = 0;		//Clear the repetition counter.
					GsmFsmState = GFSM_GETRSSI;	//Going to get the RSSI of the communication
					/*The rest of the code that runs in this state should not break. In this
					way the system enters GFSM_RSSI state at once, as this is the one that
					follows.*/
				} else {
					/*Not registered, yet. Need to repeat after a short time if there are any
					repetitions left.*/
//					SystemStatus &= ~SS_NEWDATA;	//The incoming line is parsed
					SystemStatus |= (SS_GPRSDELAY | SS_NOEXIT);
					if(TestPar == 0) {
//						Task_sleep(3000000/Clock_tickPeriod);	//Not scanning? 3 secs delay
						Semaphore_pend(GPRSSem, 3000000 / Clock_tickPeriod);
					} else {
//						Task_sleep(1000000/Clock_tickPeriod);	//Resend after one second
						Semaphore_pend(GPRSSem, 1000000 / Clock_tickPeriod);
					}
					DEBUGEXIT(129);
					break;
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_GETRSSI:				//Going to get the connection quality parameters
				DEBUGENTER(GFSM_GETRSSI);
				//Command mode
//				SystemStatus &= ~SS_NEWDATA;//Reset this flag
//				SystemStatus |= SS_RESPONSE;//Going to enter Response mode
				//Issue a Check Quality command and set the response timeout for 10 seconds
				UartParams->readTimeout = 10000000/Clock_tickPeriod;
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CSQ], 8);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 25);
				if(InChars == 0) {			//Timeout? => Error, Power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;
				}
				//Response mode of state
				//First line of response. We must fetch the registration status code
				/*Possible returns from "AT+CSQ" command are "+CSQ: <rssi>,<ber>" on success,
				or "+CME ERROR: <err>" on failure. Since we don't care about the kind of
				error in second case, we only need to check the third character to find out
				if we have an error in ME or not. Keep in mind that sometimes the module
				sends rubbish and that in the buffer the trailing "\r\nOK\r\n" should be read.
				There is always the possibility of not receiving the ending OK, so we have to
				check it.*/
				if(GsmInBuffer[4] == 'S') {
					//Correct answer, fetch rssi and ber values
					//First get the integer RSSI into GsmRSSI
					TestPar = ParseNumber(&GsmInBuffer[8], &TestUVal);
					GsmRSSI = TestUVal & 0xFF;	//Filter only the 8 lower bits
					//Next get the integer BER into GsmBER
					ParseNumber(&GsmInBuffer[8 +TestPar], &TestUVal);
					GsmBER = TestUVal & 0xFF;	//Filter only the 8 lower bits
				} else if (GsmInBuffer[4] == 'M') {
					//Received +CME ERROR => Power off
					FSMEnterPowerOff();
					DEBUGEXIT(254);
					break;					//Go there at once (Do not check for a retry)
				} else {
					//Received rubbish characters, so cannot conclude
					GsmRSSI = 100;			//Not correct RSSI value
					GsmBER = 100;			//Not correct BER value
				}
				/*Lets find out if there is the whole command response in the buffer. Find
				the first occurrence of CRLF, include both leading and trailing CRLFs and
				check if there are more characters to contain "OK". If not, we must read
				the rest!*/
				TestVal = FindCRLF(&GsmInBuffer[2], InChars -2) +4;
				if(TestVal > (InChars -6)) {
					InChars = UART_read(GSMUartHandle, &GsmInBuffer[InChars], 30);
				}
//				SystemStatus &= ~(SS_SECLINE | SS_RESPONSE);// enter Command mode, First line
				GsmRepeatCnt = 0;			//Clear the repetition counter.
				if((SystemStatus & SS_SMSRECEIVE) != 0) {
					//Need to receive new SMS
					GsmFsmState = GFSM_SETPDUSMS;	//Proceed to Set SMS Mode
					SystemStatus &= ~SS_SMSRECEIVE;	//Clear the event flag for SMS reception
					SystemStatus |= SS_NOEXIT;		//Proceed at once
					DEBUGEXIT(1);
					break;
				} else if((SystemStatus & (SS_GSMTYPEREQ | SS_HTTPSETS)) != 0) {
					//On Settings or Signal request...
					GsmFsmState = GFSM_ATTACH;	//Going to Attach to GPRS Service
					/*The rest of the code that runs in this state should not break. In this
					way the system enters GFSM_ATTACH state at once, as this is the one that
					follows. NEWDATA is still active, to force enter the following state,
					whichever it is*/
				} else {
					//else, no GPRS communication. Power off
					GsmFsmState = GFSM_POWEROFF;
					SystemStatus |= (SS_NOEXIT | SS_POWEROFF | SS_NEWDATA);
					SystemStatus &= ~(SS_GPRSDELAY | SS_GSMTYPEREQ | SS_HTTPSETS);
					break;
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_ATTACH:				//Going to attach to GPRS service
				DEBUGENTER(GFSM_ATTACH);
				//Command mode
//				SystemStatus &= ~SS_NEWDATA;//Reset this flag, going out of Attach state
//				SystemStatus |= SS_RESPONSE;//Going to enter Response mode
				//Going to issue a GPRS Attach command. Set the response time to 10 seconds
				UartParams->readTimeout = 10000000/Clock_tickPeriod;
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CGATT], 12);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 19);
				//No matter what the answer is, signal the usage of GPRS radio
				PIN_setOutputValue(ledPinHandle, GPRS_ACTIVELED, 1);
				if(InChars == 0) {			//Error => Power off the module
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;
				}
				//Response mode
				/*The command just issued is "AT+CGATT=1". We need to check its response
				first. The response can be "OK" when everything is fine, or "+CME ERROR: \
				<err>". By checking only the first character we can see if there is an error
				or not*/
				if(GsmInBuffer[2] == 'O') {	//OK =>
					GsmFsmState = GFSM_DEFPDP;		//Next state is to define PDP
//					SystemStatus &= ~SS_RESPONSE;	//Going to enter command mode
					/*The rest of the code that runs in this state should not break. In this
					way the system enters GFSM_DEFPDP state at once, as this is the one that
					follows.*/
				} else {					//On Error => Power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_DEFPDP:				//Lets define a PDP context
				DEBUGENTER(GFSM_DEFPDP);
				//Command Mode
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				/*Going to issue a complex command. The command also needs the APN of the GSM
				network.*/
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CGDCONT], 19);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, APN, StrSize(APN));
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CGDCONT] +18, 3);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 9);
				if(InChars == 0) {			//Error => Power off the module
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;
				}
				//Response mode
				/*The command "AT+CDGCONT" returns "OK" on success or "ERROR" on failure. By
				checking only the first character we are able to realize if it was successful
				or not.*/
				if(GsmInBuffer[2] == 'O') {	//OK =>
					GsmFsmState = GFSM_STTASK;		//Next state is to start a GPRS task
//					SystemStatus &= ~SS_RESPONSE;	//Enter command mode
					/*The rest of the code that runs in this state should not break. In this
					way the system enters GFSM_STTASK state at once, as this is the one that
					follows.*/
				} else {					//On Error => Power off the module
					FSMEnterPowerOff();
					DEBUGEXIT(254);
					break;
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_STTASK:				//Going to set APN, Username/Password for GPRS
				DEBUGENTER(GFSM_STTASK);
				//Command Mode
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				/*Another complex command is going to be sent to the GSM module. It also needs
				APN, Username and Password of the Network Provider's GPRS service.*/
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CSTT], 9);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, APN, StrSize(APN));
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CSTT] +8, 3);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, Username, StrSize(Username));
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CSTT] +8, 3);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, Password, StrSize(Password));
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CSTT] +10, 3);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 9);
				if(InChars == 0) {			//Error? => Power off the module
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;
				}
				//Response mode
				/*The command "AT+CSTT" returns "OK" on success or "ERROR" on failure. By
				checking only the first character we are able to realize if it was successful
				or not.*/
				if(GsmInBuffer[2] == 'O') {	//OK =>
					GsmFsmState = GFSM_CONNON;	//Next state: Bring wireless communication on
//					SystemStatus &= ~SS_RESPONSE;	//Enter command mode
					/*The rest of the code that runs in this state should not break. In this
					way the system enters GFSM_CONNON state at once, as this is the one that
					follows.*/
				} else {					//On Error =>
					FSMEnterPowerOff();		// Just power off the module
					DEBUGEXIT(254);
					break;
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_CONNON:				//Going to bring wireless connection on
				DEBUGENTER(GFSM_CONNON);
				//Command Mode
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				/*Going to issue a AT+CIICR command. It may take a long time until this
				happens, so set the response timeout to 60 seconds.*/
				UartParams->readTimeout = 60000000/Clock_tickPeriod;
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CIICR], 10);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 9);
				if(InChars == 0) {			//Timeout? => Power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;
				}
				//Response mode
				/*The command "AT+CIICR" returns "OK" on success or "ERROR" on failure. By
				checking only the first character we are able to realize if it was successful
				or not.*/
				if(GsmInBuffer[2] == 'O') {	//OK =>
					GsmFsmState = GFSM_GETIP;		//Next state is to get our IP address
//					SystemStatus &= ~SS_RESPONSE;	//Enter command mode
					/*The rest of the code that runs in this state should not break. In this
					way the system enters GFSM_GETIP state at once, as this is the one that
					follows.*/
				} else {					//On Error =>
					FSMEnterPowerOff();		// Power off the module
					DEBUGEXIT(254);
					break;
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_GETIP:				//Going to get local IP address
				DEBUGENTER(GFSM_GETIP);
				//Command Mode
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				//Issue a Get IP address. Expect an answer within 10 seconds
				UartParams->readTimeout = 10000000/Clock_tickPeriod;
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CIFSR], 10);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 19);
				if(InChars == 0) {			//Timeout? =>
					FSMEnterPowerOff();		//Go to Power Off Module state
					DEBUGEXIT(255);
					break;
				}
				//Response mode
				/*The command "AT+CIFSR" returns an IP address on success or "ERROR" on
				failure. By checking only the second character we are able to realize if it
				was successful or not. The reason for the second character is that if the
				provider assigns an IPv6 address, the first character is valid for both IPv6
				and ERROR; 'E' is a valid IPv6 character. Also keep in mind the CRLF sequence
				at the beginning of the answer.*/
				if(GsmInBuffer[3] == 'R') {	//Error => Proceed to power off
					FSMEnterPowerOff();
					DEBUGEXIT(254);
					break;
				} else {					//Valid IP address
					GsmFsmState = GFSM_ADDIPHEAD;	//Next state is for adding IP header
//					SystemStatus &= ~SS_RESPONSE;	//Enter command mode
					/*The rest of the code that runs in this state should not break. In this
					way the system enters GFSM_ADDIPHEAD state at once, as this is the one
					that follows.
					eliaschr@NOTE: If we need to keep the IP address, this is the point to do
					it! Don't use the first two characters, as these are a CRLF and then parse
					the rest of the input line from offset 2 and on.*/
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_ADDIPHEAD:			//Going to add IP header to received data
				DEBUGENTER(GFSM_ADDIPHEAD);
				//Command Mode
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				//Add IP header to every incoming packet!
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CIPHEAD], 14);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 9);
				if(InChars == 0) {			//Timeout? => Bad thing...
					FSMEnterPowerOff();		//Power off the module
					DEBUGEXIT(255);
					break;
				}
				//Response mode
				/*The command "AT+CIPHEAD" returns "OK" on success or "ERROR" on failure. By
				checking only the first character we are able to realize if it was successful
				or not.*/
				if(GsmInBuffer[2] == 'O') {	//OK =>
					GsmFsmState = GFSM_STARTTCP;	//Next state is to Start a TCP connection
//					SystemStatus &= ~SS_RESPONSE;	//Enter command mode
					/*The rest of the code that runs in this state should not break. In this
					way the system enters GFSM_STARTTCP state at once, as this is the one
					that follows.*/
				} else {					//On Error =>
					FSMEnterPowerOff();		//Oooops... Proceed to power off the module
					DEBUGEXIT(254);
					break;
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_STARTTCP:				//Lets start a TCP connection to server
				DEBUGENTER(GFSM_STARTTCP);
				//Command Mode
//				SystemStatus &= (~SS_NEWDATA | SS_SECLINE);//Clear NEWDATA flag and expect the
											// first line of its response
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				/*Going to issue another complex command. Now server's host address and its
				port are needed. The timeout is still set to 10 seconds.*/
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CIPSTART], 19);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, Host, StrSize(Host));
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CIPSTART] +16, 3);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, Port, StrSize(Port));
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CIPSTART] +18, 3);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 19);
				if(InChars == 0) {			//Timeout occurred? =>
					FSMEnterPowerOff();		//Proceed to power off state
					DEBUGEXIT(255);
					break;
				}
				//Response mode
				/*The command "AT+CIPSTART" returns two lines. The first can be "OK" on
				success or "+CME ERROR: <err>" on failure. The second one can be one of
				"ALREADY CONNECT", "CONNECT OK", "CONNECT FAIL", or "STATE: <stat>". For the
				first line we only need to check the first character in order to find out if
				the command was successful*/
				//First line of response. It can be OK or +CME ERROR:<err>
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
				if(GsmInBuffer[2] != 'O') {	//Error =>
					FSMEnterPowerOff();		//Power off the module
					DEBUGEXIT(254);
					break;
				} else {
//					SystemStatus |= SS_SECLINE;//Next time will get the second line of the
											// response
					tmpLen = 6;				//OK response is 6 characters long
					/*Going to remove current line from the buffer and ensure we have one
					complete line in it.*/
					for(TestVal = 0; TestVal < (InChars - tmpLen); TestVal++) {
						GsmInBuffer[TestVal] = GsmInBuffer[tmpLen++];
					}
					InChars = TestVal;		//Refresh InChars to contain the number of
											// characters contained now in the buffer
					tmpLen = FindCRLF(&GsmInBuffer[2], InChars -2);
					if(tmpLen < 0){			//Erroneous characters found? =>
						FSMEnterPowerOff();	//Power off!
						DEBUGEXIT(253);
						break;
					}
					/*There is the possibility of not received a complete CRLF, so we check it
					and if not, we read the second character of it.*/
					if(tmpLen == (InChars -3)) {
						InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars], 1);
					} else if(tmpLen == 0){
						InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars],
							19 -InChars);
					}
				}
				/*Second line of response. Can be "ALREADY CONNECT", "CONNECT OK",
				"CONNECT FAIL", or "STATE: <stat>". By checking only the ninth character of
				the response, we know if we have a connection or not.*/
//				SystemStatus &= ~SS_SECLINE;	//Be prepared for first line at our next
											// response
				if(GsmInBuffer[10] != 'O') {//The only character we need to check is the ninth
											// one. If it is not 'O' then we have an error
					FSMEnterPowerOff();
					DEBUGEXIT(252);
					break;
				} else {					//Connection OK
					GsmFsmState = GFSM_PREPDATA;	//Next state is the preparation of data
//					SystemStatus &= ~SS_RESPONSE;	//Command mode for the next state
					/*The rest of the code that runs in this state should not break. In this
					way the system enters GFSM_PREPDATA state at once, as this is the one that
					follows.*/
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_PREPDATA:				//Going to prepare data to be sent
				DEBUGENTER(GFSM_PREPDATA);
				//Command Mode
//				SystemStatus &= (~SS_NEWDATA | SS_SECLINE);//Clear NEWDATA flag and expect the
											// first line of its response
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				//Going to issue a Send Data command. The timeout response is set to 20 secs
				UartParams->readTimeout = 20000000/Clock_tickPeriod;
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CIPSEND], 12);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 4);
				if(InChars == 0) {			//Timeout? => Power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;
				}
				//Response mode
				//The expected response is "\r\n> ", without trailing CRLF
				if(GsmInBuffer[2] != '>') {	//Erroneous response? =>
					FSMEnterPowerOff();		//Power off the module
					DEBUGEXIT(254);
					break;
				}
				GsmRepeatCnt = 0;			//Reset the number of retries for HTTP request
				GsmFsmState = GFSM_INSDATA;	//Next state is to insert data to the request
//				SystemStatus &= ~SS_RESPONSE;	//Next time enter command mode
				/*The rest of the code that runs in this state should not break. In this way
				the system enters GFSM_INSDATA state at once, as this is the one that
				follows.*/
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_INSDATA:				//Send request to server.
				DEBUGENTER(GFSM_INSDATA);
				//Command Mode
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				// Need to send the correct request according to input parameter Type
				if ((SystemStatus & SS_GSMTYPEREQ) == 0) {	//Settings type request?
					//Going to issue a "settings' request to server
					InChars = UART_write(GSMUartHandle, HTTPStrs[HTTP_GETSET], 11);
				} else {
					InChars = UART_write(GSMUartHandle, HTTPStrs[HTTP_GETSIG], 15);
				}
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				if(CurrNode == DEF_MASTER_ADDRESS) {
					//The request will be for the master node so send its UID
					InChars = UART_write(GSMUartHandle, UID, StrSize(UID));//Device UID
				} else {					//... or ...
					//The request will be for a slave node, so send node's MAC address
					NodePtr = GetNodeEntry(CurrNode);
					for(tmpLen = 0; tmpLen < 8; tmpLen++) {
						Byte2Hex(NodePtr->MACAddr[tmpLen], &GsmInBuffer[tmpLen *2]);
					}
					InChars = UART_write(GSMUartHandle, GsmInBuffer, 16);
				}
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				if ((SystemStatus & SS_GSMTYPEREQ) != 0) {	//Signal type request?
					/*The battery voltage Vbatt is the A/D converter's value. The battery
					voltage comes through a divider resistor network with values 120K and 51K.
					The output of the network is the voltage value across 51K resistor and is
					the voltage value that drives the A/D. The A/D samples this voltage using
					a 1.5V reference and the resulting value is 12 bits long. Thus, to
					calculate the real battery voltage we have:
					from divider resistor network: Vbattery/Vadc = (120K+51K)/51K =>
					Vbattery = Vadc * 171K/51K => Vbattery = Vbatt * (1.5 /4096) * (171 /51),
					where Vbatt is the number representation of A/D.
					finally: Vbattery = Vbatt * 3.3529411 * 1.5 /4096
					Vbattery is stored in local Voltage variable.
					*/
					if(CurrNode == DEF_MASTER_ADDRESS) {
						Vbatt = VBattLvl;
					} else {
						Vbatt = NodesParsArr[CurrNodeIdx].Battery;
					}
					float Voltage = (3.353*1.5/4096) * (float)Vbatt;
					if(Voltage<3.3) Voltage=3.3;	//Truncate minimum value
					if(Voltage>4.201) Voltage = 4.201;//Truncate maximum value
					/*At 3.5V the battery is considered totally empty, while over 4.2V the
					battery is considered totally full. So to calculate the percentage of the
					charge: */
					ChargePercent = (int)(((Voltage - 3.5) / 0.7) * 100);
					if (ChargePercent < 1 ) ChargePercent=0;//Truncate lower level

					//Next, lets calculate the current time and when is the next update
					//Store date time in TimeBuffer
					NowTime = RTCGetCurrTime();
					RTCtoDateStr(NowTime, TimeBuffer, true);
					//Prepare NextUpdateTime by adding UpdateTime to current date/time
					RTCtoDateStr(NowTime + (UpdateTime *60), NextUpdateTime, true);
//					SetNextUpdateTime(UpdateTime);

					//Finally, we need to find out how many timestamps we will send
					MaxCounter = STAMPTHRESHOLD;
					//GSM On counters are not in use anymore. They stay in the code only for
					// backwards compatibility
					GSMonCounter = 0;
					GSMonMCounter = 0;
					if((CurrNode == DEF_MASTER_ADDRESS) ||
						(CurrNode == DEF_BROADCAST_ADDRESS)) {
						StampsCounter = 0;
						RychoMasterCounter = 0;
						SendTimes = SndTm;
						NodeRSSI = GsmRSSI;
						NodeBER = GsmBER;
					} else {
						StampsCounter = NodesParsArr[CurrNodeIdx].StampsNo;
						RychoMasterCounter = NodesParsArr[CurrNodeIdx].MasterCounter;
						SendTimes = NodesParsArr[CurrNodeIdx].SndTm;
						NodeRSSI = NodesParsArr[CurrNodeIdx].rssi;
						NodeBER = 100;
					}
					if(MaxCounter > StampsCounter) {
						MaxCounter = StampsCounter;
					}

					//Now, lets form the whole request to issue
					InChars = UART_write(GSMUartHandle, HTTPStrs[HTTP_ST], 6);
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					InChars = UART_write(GSMUartHandle, TimeBuffer, 21);	//Current time
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					//Append master counter of total events
					sprintf(GsmInBuffer, "&MCnt=%lu", RychoMasterCounter);
					InChars = UART_write(GSMUartHandle, GsmInBuffer, StrSize(GsmInBuffer));
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					//Set the number of timestamps in this request (calculated earlier)
					sprintf(GsmInBuffer, "&SCnt=%u", MaxCounter);
					InChars = UART_write(GSMUartHandle, GsmInBuffer, StrSize(GsmInBuffer));
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					//Set the events recorder during GSM module on since last successful
					//communication
					sprintf(GsmInBuffer, "&CntG=%lu", GSMonCounter);
					InChars = UART_write(GSMUartHandle, GsmInBuffer, StrSize(GsmInBuffer));
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					//Set the total number of events recorded during GSM module on
					sprintf(GsmInBuffer, "&MCntG=%lu", GSMonMCounter);
					InChars = UART_write(GSMUartHandle, GsmInBuffer, StrSize(GsmInBuffer));
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					//Append the percentage of battery charge (calculated earlier)
					sprintf(GsmInBuffer, "&Bat=%u", ChargePercent);
					InChars = UART_write(GSMUartHandle, GsmInBuffer, StrSize(GsmInBuffer));
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					//Number of successful communications
					sprintf(GsmInBuffer, "&SdTm=%u", SendTimes);
					InChars = UART_write(GSMUartHandle, GsmInBuffer, StrSize(GsmInBuffer));
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					//Append the voltage of the battery (calculated earlier)
					sprintf(GsmInBuffer, "&Vlt=%.3f", Voltage);
					InChars = UART_write(GSMUartHandle, GsmInBuffer, StrSize(GsmInBuffer));
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					//Append NextUpdateTime value
					sprintf(GsmInBuffer, "&Nupd=%s", NextUpdateTime);
					InChars = UART_write(GSMUartHandle, GsmInBuffer, StrSize(GsmInBuffer));
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					//Append RSSI value
					sprintf(GsmInBuffer, "&Csq=%i", NodeRSSI);
					InChars = UART_write(GSMUartHandle, GsmInBuffer, StrSize(GsmInBuffer));
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					//Append BER value
					sprintf(GsmInBuffer, "&Ber=%i", NodeBER);
					InChars = UART_write(GSMUartHandle, GsmInBuffer, StrSize(GsmInBuffer));
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					//Lets append the timestamp records. The number of records can be up to
					// STAMPTHRESHOLD
					InChars = UART_write(GSMUartHandle, HTTPStrs[HTTP_STMP], 6);
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;				//Exit the state
					}
					/*The maximum number of timestamps to be sent is defined by
					STAMPTHRESHOLD. But if there is another reason that made us communicate to
					server, we have to check if there are so many timestamps in the buffer. If
					there are less than that, we need to send only as many as the buffer
					contains.*/
					StampsSent = MaxCounter;//Need to keep the number of stamps we try to send
					CurrParam = 0;			//Going to start searching from the beginning of
											// the events array
					GsmInBuffer[0] = '|';
					for(StampsCounter = 0; StampsCounter < MaxCounter; StampsCounter++) {
						while((EvArray[CurrParam].NodeShortAddr != CurrNode) &&
							(CurrParam < MAXNODEEVENTS)) {
							CurrParam++;
						}
//						if(CurrParam == MAXNODEEVENTS) {
//							break;
//						}
						RTCtoDateStr(EvArray[CurrParam].timestamp, &GsmInBuffer[1], true);
						sprintf(&GsmInBuffer[22], ";%.1f;%1i\0",
							(((float)(((long)EvArray[CurrParam].temperature -
								NodesParsArr[CurrNode].CALADC12_15V_30C) * (85 - 30)) /
								(NodesParsArr[CurrNode].CALADC12_15V_85C -
									NodesParsArr[CurrNode].CALADC12_15V_30C) + 30.0f) -2.5),
							//Here we add the record type which is hidden in the month value
							EvArray[CurrParam].eventType);
						InChars = UART_write(GSMUartHandle,
							&GsmInBuffer[(StampsCounter > 0) ? 0 : 1],
							StrSize(GsmInBuffer) -((StampsCounter > 0) ? 0 : 1));
						if(InChars == UART_ERROR) {	//If there was a timeout during write, the
							// return value is UART_ERROR. In that case we have to gracefully
							// exit.
							// => Error. Need to power off
							FSMEnterPowerOff();
							DEBUGEXIT(255);
							break;			//Exit the state
						}
						CurrParam++;		//Check next event in the events list
					}
				}							//End of GSM_SIGNAL type of request

				//The rest of the request is common to both Settings and Signal types
				InChars = UART_write(GSMUartHandle, HTTPStrs[HTTP_HEAD], 17);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, Host, StrSize(Host));
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, HTTPStrs[HTTP_END], 24);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				/*At this point, the last character received was the space character from "> "
				sequence. When we terminate the data to be sent using CTRL-Z character, the
				module sends a newline character. We want to skip that. Also, the timeout of
				the response is set to 30 seconds.*/
				UartParams->readTimeout = 30000000/Clock_tickPeriod;
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 13);
				if(InChars == 0) {			//Timeout? => Well...
					FSMEnterPowerOff();		//Proceed to power off the module
					DEBUGEXIT(255);
					break;
				}
				//Response mode
				/*The previous command that sends a GET request ("AT+CIPSEND") responds with
				"SEND OK" or "SEND FAIL". By checking only the sixth character we can see if
				the command was successful or not.*/
				if(GsmInBuffer[7] != 'O') {	//The only character we need to check is the sixth
					if(++GsmRepeatCnt >= HTTP_MAXREPEAT) {
						//Maximum repetition times reached without success...
						FSMEnterPowerOff();	//Proceed to power off state
						DEBUGEXIT(254);
						break;
					}
					//OK. We can try once more. Do it without exiting the FSM loop
					SystemStatus |= SS_NOEXIT;
					DEBUGEXIT(128);
					break;
				} else {
					/*We must remove this line from GSM buffer and leave a complete line of
					data in there. The next state will handle them.*/
					GsmRepeatCnt = 0;		//Clear the repetition counter
					for(tmpLen = 0; tmpLen < (InChars -11); tmpLen++) {
						GsmInBuffer[tmpLen] = GsmInBuffer[tmpLen +11];
					}
					InChars = tmpLen;		//Refresh the number of characters in buffer
					//Ensure we have 13 characters in buffer "\r\n+IPD,xxxx:H..."
					UartParams->readTimeout = 10000000/Clock_tickPeriod;
					InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars], 13);
					GsmFsmState = GFSM_FINDIPD;		//Lets find the response length
//					SystemStatus &= ~SS_RESPONSE;	//in command mode
					/*The rest of the code that runs in this state should not break. In this
					way the system enters GFSM_FINDIPD state at once, as this is the one that
					follows.*/
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_FINDIPD:				//Going to find the server's response length
				DEBUGENTER(GFSM_FINDIPD);
				//Command Mode
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				/*The data send from server are a lot of info and many lines. The first part
				is "+IPD,<length>:". The "<length> specifies the number of characters the
				server sends back to us. So, we split the response into two blocks; the first
				expects the "+IPD,<length>:" part and decodes the <length> number, the second
				just receives as many bytes as specified by the latter parameter and then
				parses the parameters needed. Lets find the "+IPD," part. Using our setup it
				should be the first in the buffer, after the leading CRLF.*/
				if(strncmp(&GsmInBuffer[2], IPDLen, 5) != 0) {
					FSMEnterPowerOff();		//Power off because +IPD was not found
					DEBUGEXIT(255);
					break;
				}
				//Response mode
				GsmFsmState = GFSM_GETLENGTH;	//Going to get the length parameter
//				SystemStatus &= ~SS_RESPONSE;	//Next state will be entered in command mode
				/*The rest of the code that runs in this state should not break. In this way
				the system enters GFSM_GETLENGTH state at once, as this is the one that
				follows.*/
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_GETLENGTH:			//Going to get the <length> parameter of +IPD:...
				DEBUGENTER(GFSM_GETLENGTH);
				//Command Mode
				//Get the number of characters of server's response
				tmpLen = ParseNumber(&GsmInBuffer[7], &GsmLength);
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				//Response mode
				/*Need to prepare the incoming data for fetching and parsing. Delete the
				+IPD part of line and leave only pure data fetched from server.*/
				tmpLen += 7;				//Must add the number of characters of "\r\n+IPD,"
				for(TestVal = 0; TestVal < (InChars -tmpLen); TestVal++) {
					GsmInBuffer[TestVal] = GsmInBuffer[TestVal +tmpLen];
				}
				InChars = TestVal;

				GsmFsmState = GFSM_GETDATA;	//Going to get data from server
//				SystemStatus &= ~SS_RESPONSE;	//Next state enters the command mode
				/*The rest of the code that runs in this state should not break. In this way
				the system enters GFSM_GETDATA state at once, as this is the one that follows.
				*/
				if(GsmLength == 0) {		//No data to parse?
					// ... then need to close the connection without exiting the FSM loop
					/*Perhaps we need to send a new GET request for a new node instead of
					closing the connection.*/
					if((CurrNode -1) >= NodesCnt) {
//						SystemStatus &= ~SS_RESPONSE;
//						GsmFsmState = GFSM_CLOSE;
//						GsmFsmState = GFSM_DEACT;
						GsmFsmState = GFSM_POWEROFF;
						/*eliaschr@NOTE: Just remember that we need to set SS_NEWDATA if the
						communication was OK and we finally just need to power off... Only in
						an error case SS_NEWDATA stays low while transferring control to Power
						Off state.*/
					} else {
						CurrNode++;			//Going to issue request for the next node
						CurrNodeIdx = CurrNode -2;
						GsmFsmState = GFSM_PREPDATA;
						SystemStatus |= SS_NOEXIT;
						DEBUGEXIT(3);
						break;
					}
					//The following flags depend on the following termination state,
					// GFSM_CLOSE, GFSM_DEACT or GFSM_POWEROFF
					SystemStatus |= (SS_NOEXIT | SS_POWEROFF | SS_NEWDATA);
					DEBUGEXIT(255);
					break;
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_GETDATA:				//Going to get <length> data bytes and filter them
				DEBUGENTER(GFSM_GETDATA);
				//Command Mode
//				SystemStatus &= ~(SS_NEWDATA | SS_FOUNDALL);//Clear NEWDATA and FOUNDALL flags
				SystemStatus &= ~SS_FOUNDALL;//Clear the Found All Parameters flag
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				/*Set the maximum number of characters to be read according to <length> value
				just read.*/
				GsmLimitChrs = GsmLength -InChars;
				//Clear the array of searched parameters
				for(TestPar = 0; TestPar < PARAMSNO; TestPar++) {
					ParamSearch[TestPar].offset = 0;
					ParamSearch[TestPar].status = PS_SCAN;
				}
				ParamsFound = 0;			//No parameters parsed yet
				/*eliaschr@TODO: Need to exit the loop of fetching and parsing data, perhaps
				if tmpLen = 0 after the UART_read*/
//				UartParams->readTimeout = 10000000/Clock_tickPeriod;
				if((InChars < GSMINSIZE) && (GsmLimitChrs > 0)){
					tmpLen = UART_read(GSMUartHandle, &GsmInBuffer[InChars],
						(GSMINSIZE <= GsmLimitChrs) ? (GSMINSIZE -InChars) :
							(GsmLimitChrs -InChars));
					InChars += tmpLen;
					GsmLimitChrs -= tmpLen;
				}
				//Response mode
				GsmFsmState = GFSM_PARSEDATA;//Next state is to parse incoming data
				//The next state is a Response only state, so do not clear RESPONSE flag
				/*The rest of the code that runs in this state should not break. In this way
				the system enters GFSM_GETDATA state at once, as this is the one that follows.
				*/
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_PARSEDATA:			//Going to parse incoming data, until finished
				DEBUGENTER(GFSM_PARSEDATA);
				/*The system needs to read all data before going to power off the module.
				Parsing those data could involve header parsing and pure data sent from
				server. In our case data came from the server through header are irrelevant so
				the only things we need to find are the parameters we need to alter in our
				variables.*/
//				SystemStatus &= ~(SS_ALLNOTFOUND + SS_NEWDATA);	//Clear the "All Not Found"
//											// and the "New Data" flags
				SystemStatus &= ~SS_ALLNOTFOUND;//Clear the "All Not Found" flag
				CharCnt = 0;				//Reset the counter of characters in GsmInBuffer
				NFCount = ParamsFound;		//Reset the counter of parameters not found
				/*The parsing of variables is done for the whole input line. The idea is that
				the system searches for variables. Every time one is not found it is not
				searched anymore for this line. If there is no variable found then it
				proceeds to the next incoming line of data. Every time a parameter is found,
				it is flagged as FOUND and not searched any more through the rest of incoming
				lines. When all parameters are found the system waits all the data to be
				received from the GSM module and then it powers it off (or sends another HTTP
				request to server). The only rule we use is that a parameter should start from
				the beginning of the input line, and if there are other parameters in the same
				line, they should follow each other using only one delimiter character.*/
				while((CharCnt < InChars) &&
					((SystemStatus & (SS_ALLNOTFOUND | SS_FOUNDALL)) == 0) &&
					(GsmInBuffer[CharCnt] != '\r') && (GsmInBuffer[CharCnt] != '\n')) {
					//If we have more characters in input line and there are more parameters
					// to be scanned, then for each one do:
					for(TestPar = 0; TestPar < PARAMSNO; TestPar++) {
						//When the current parameter is still in SCAN mode
						if(ParamSearch[TestPar].status == PS_SCAN) {
							InChr = ParamsArr[TestPar].str[ParamSearch[TestPar].offset];
							//Check if the currently scanned character a terminating one
							if(InChr == '\0') {
								//If yes, we must mark the current parameter as FOUND
								ParamSearch[TestPar].status = PS_FOUND;
								ParamsFound++;//Increase the number of found parameters
								/*When a parameter is found, its associated function should be
								called to parse its value. The function returns the number of
								characters used for the parameter's value. We must skip these
								characters and not scanned for the parameter searching,
								including the delimiter one.*/
								CharCnt += ParamsArr[TestPar].func(CharCnt, CurrNode);// +1;
								if(ParamsFound >= PARAMSNO) {
									/*In case that all parameters were found, flag this
									condition. No need to scan anymore.*/
									SystemStatus |= SS_FOUNDALL;
									break;	//Exit the "for" loop
								}
								/*If there is any parameter that is not parsed yet, all the
								parameters not found, together with those that are still in
								scan mode must get ready to be scanned again.*/
								for(TestVal = 0; TestVal < PARAMSNO; TestVal++) {
									if(ParamSearch[TestVal].status == PS_NOTFOUND) {
										ParamSearch[TestVal].status = PS_SCAN;
									} else if(ParamSearch[TestVal].status == PS_SCAN) {
										ParamSearch[TestVal].offset = 0;
									}
								}
								/*For the "Not Found" counter we must exclude the number of
								parameters already found.*/
								NFCount = ParamsFound;
//								TestPar = 0;//Reset the parameter number to scan all remaining
											// parameters from the very beginning
								break;		//Terminate the "for" loop
							} else if(GsmInBuffer[CharCnt] != InChr) {
								/*In case the input character differs from the current one of
								the currently scanned parameter, then we must flag it as NOT
								FOUND.*/
								ParamSearch[TestPar].status = PS_NOTFOUND;
								ParamSearch[TestPar].offset = 0;
								NFCount++;	//Increase the number of not found parameters
								if(NFCount == PARAMSNO) {
									//Did we scan all of the parameters left and did not
									// find them? => skip the rest of the line
									SystemStatus |= SS_ALLNOTFOUND;
									//Clear the array of searched and not found parameters
									for(TestVal = 0; TestVal < PARAMSNO; TestVal++) {
										if(ParamSearch[TestVal].status == PS_NOTFOUND) {
											ParamSearch[TestVal].status = PS_SCAN;
										}
									}
									break;	//Terminate the "for" loop
								}
							} else {		//Current character matches the current character
											// of the currently searching parameter
								ParamSearch[TestPar].offset++;//Next time test the next one
							}
						}					//End of "if" block for checking only parameters
											// in PS_SCAN status and skip the rest
					}						//End of "for" loop for checking all parameters
					CharCnt++;				//Going to check next character of input line
				}							//End of "while" loop that checks for all
											// characters in input line
				//Going to repeat the process, so reset all not found parameters to scan mode
				for(TestVal = 0; TestVal < PARAMSNO; TestVal++) {
					if(ParamSearch[TestVal].status == PS_NOTFOUND) {
						ParamSearch[TestVal].status = PS_SCAN;
					}
				}
				//Do we have more characters in the buffer to parse?
				if(CharCnt < InChars) {		//Yes =>
					//Skip the rest of the line (normal characters plus CRLFs)
					do {
						tmpLen = FindCRLF(&GsmInBuffer[CharCnt], InChars -CharCnt);
						if(tmpLen > 0) {
							CharCnt += tmpLen;
						}
						tmpLen = SkipCRLF(&GsmInBuffer[CharCnt], InChars -CharCnt);
						if(tmpLen > 1) {	//Full CRLF found
							CharCnt += tmpLen;
							break;
						} else {
							InChars = UART_read(GSMUartHandle, GsmInBuffer,
								(GsmLimitChrs > GSMINSIZE) ? GSMINSIZE : GsmLimitChrs);
							GsmLimitChrs -= InChars;
							CharCnt = SkipCRLF(GsmInBuffer, InChars);
							if(CharCnt > 0) {
								for(tmpLen = 0; tmpLen < (InChars -CharCnt); tmpLen++) {
									GsmInBuffer[tmpLen] = GsmInBuffer[tmpLen +CharCnt];
								}
								InChars = tmpLen;
							}
						}
					} while (GsmLimitChrs > 0);
					//Discard the line of CharCnt characters
					if(CharCnt != 0) {
						for(tmpLen = 0; tmpLen < (InChars - CharCnt); tmpLen++){
							GsmInBuffer[tmpLen] = GsmInBuffer[tmpLen +CharCnt];
						}
						InChars = tmpLen;
						/*If there are more characters to be read from the HTTP response,
						read as many of them as you can.*/
						tmpLen = GSMINSIZE -InChars;
						if(tmpLen > GsmLimitChrs) {
							tmpLen = GsmLimitChrs;
						}
						if(tmpLen > 0) {
							tmpLen = UART_read(GSMUartHandle, &GsmInBuffer[InChars], tmpLen);
							/*No need to check for timeout, as there are surely more
							characters expected. Even in case of a timeout the rest of the
							code will handle it without problem.*/
							GsmLimitChrs -= tmpLen;
							InChars += tmpLen;
						}
//						CharCnt = 0;		//It is also reset during re-entrance of the state
					}
					SystemStatus |= SS_NOEXIT;
					DEBUGEXIT(1);
					break;					//Re-enter the state at once to check the next
											// line
				}
				if(GsmLimitChrs != 0) {		//Expecting more characters from response
					InChars = UART_read(GSMUartHandle, GsmInBuffer,
						(GsmLimitChrs > GSMINSIZE) ? GSMINSIZE : GsmLimitChrs);
					GsmLimitChrs -= InChars;
					CharCnt = SkipCRLF(GsmInBuffer, InChars);
					if(CharCnt > 0) {
						for(tmpLen = 0; tmpLen < (InChars -CharCnt); tmpLen++) {
							GsmInBuffer[tmpLen] = GsmInBuffer[tmpLen +CharCnt];
						}
						InChars = tmpLen;
					}
				} else {					//Response +IPD part ends here.
					/*Expecting another +IPD, or CLOSE. If a timeout occurs then we must
					close the connection manually.*/
					UartParams->readTimeout = 2000000/Clock_tickPeriod;
					InChars = UART_read(GSMUartHandle, GsmInBuffer, 12);
					UartParams->readTimeout = 1000000/Clock_tickPeriod;
					if((InChars != 0) && (GsmInBuffer[2] == '+')) {
						//We have a new +IPD...
						GsmFsmState = GFSM_FINDIPD;
						SystemStatus |= SS_NOEXIT;
						DEBUGEXIT(2);
						break;
					}
					/*Perhaps we need to send a new GET request for a new node instead of
					closing the connection.*/
					if((CurrNode -1) >= NodesCnt) {
//						SystemStatus &= ~SS_RESPONSE;
//						GsmFsmState = GFSM_CLOSE;
//						GsmFsmState = GFSM_DEACT;
						GsmFsmState = GFSM_POWEROFF;
						/*eliaschr@NOTE: Just remember that we need to set SS_NEWDATA if the
						communication was OK and we finally just need to power off... Only in
						an error case SS_NEWDATA stays low while transferring control to Power
						Off state.*/
					} else {
						CurrNode++;			//Going to issue request for the next node
						CurrNodeIdx = CurrNode -2;
						GsmFsmState = GFSM_PREPDATA;
						SystemStatus |= SS_NOEXIT;
						DEBUGEXIT(3);
						break;
					}
					//The following flags depend on the following termination state,
					// GFSM_CLOSE, GFSM_DEACT or GFSM_POWEROFF
					SystemStatus |= (SS_NOEXIT | SS_POWEROFF | SS_NEWDATA);
					DEBUGEXIT(255);
					break;
				}
//				DEBUGEXIT(0);

			//--------------------------------------
//			case GFSM_CLOSE:				//Going to close this request
//				DEBUGENTER(GFSM_CLOSE);
//				//Command Mode
////			SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
////			SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
//				UartParams->readTimeout = 5000000/Clock_tickPeriod;
//				UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CIPCLOSE], 15);
//				InChars = UART_read(GSMUartHandle, GsmInBuffer, 12);
//				if(InChars == 0) {
//					FSMEnterPowerOff();
//					DEBUGEXIT(255);
//					break;
//				}
//				//Response mode
//				/*The command "AT+CIPCLOSE" returns "CLOSE OK" on success or "ERROR" on
//				failure. By checking only the first character we are able to realize if it was
//				successful or not */
//				if(GsmInBuffer[2] == 'C') {	//CLOSE OK =>
//					//Next state is to bring wireless communication off
//					GsmFsmState = GFSM_DEACT;
////				SystemStatus &= ~SS_RESPONSE;//Enter command mode
//					/*The rest of the code that runs in this state should not break. In this
//					way the system enters GFSM_DEACT state at once, as this is the one that
//					follows.*/
//				} else {							//On Error =>
//					FSMEnterPowerOff();
//					DEBUGEXIT(254);
//					break;
//				}
//				DEBUGEXIT(0);

			//--------------------------------------
//			case GFSM_DEACT:				//Going to terminate GPRS connection
//				DEBUGENTER(GFSM_DEACT);
//				//Command Mode
////			SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
////			SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
//				UartParams->readTimeout = 5000000/Clock_tickPeriod;
//				UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CIPSHUT], 12);
//				InChars = UART_read(GSMUartHandle, GsmInBuffer, 12);
//				if(InChars == 0) {
//					FSMEnterPowerOff();
//					DEBUGEXIT(255);
//					break;
//				}
//				//Response mode
//				/*The command "AT+CIPSHUT" returns "SHUT OK" on success or "ERROR" on
//				failure. By checking only the first character we are able to realize if it was
//				successful or not. Well, no need to check anything. The next state is Power
//				Off the module*/
//				GsmFsmState = GFSM_POWEROFF;
//				//Also flag that the communication was successful.
//				SystemStatus |= (SS_NEWDATA | SS_NOEXIT | SS_POWEROFF);
////			SystemStatus &= ~SS_RESPONSE;	//Enter command mode
//				DEBUGEXIT(0);
//				break;

			//--------------------------------------
			case GFSM_ASKIMEI:				//Gets the IMEI of the module
				DEBUGENTER(GFSM_ASKIMEI);
				//Command mode of this state
//				SystemStatus &= ~SS_NEWDATA;//Clear this flag
//				SystemStatus |= (SS_SECLINE | SS_RESPONSE);//Next time we will receive the
											// first line of the IMEI response
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_GSN], 8);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 25);
				if(InChars == 0) {			//Timeout? => something's wrong
					FSMEnterPowerOff();		//Proceed to power off the module
					DEBUGEXIT(255);
					break;
				}
				//Response mode of this state, new data line received
				//First line of response, returns the IMEI. We must put a string termination
				// at its end
				for(TestPar = 2; TestPar < InChars; TestPar++) {
					if(GsmInBuffer[TestPar] <= ' ') {
						GsmInBuffer[TestPar] = '\0';
						break;
					}
				}
//				SystemStatus &= ~(SS_SECLINE + SS_NEWDATA);
				StoreUID(&GsmInBuffer[2]);	//Store the UID in Flash memory
				//Second line of response is just "OK"
				GsmFsmState = GFSM_COMMOK;	//Next state is to establish communication
											// with the telephony network provider
				SystemStatus &= ~SS_RESPONSE;	//Command mode for the next state
				SystemStatus |= SS_NOEXIT;	//Do not exit FSM, enter the new state at once
				DEBUGEXIT(0);
				break;

			//--------------------------------------
			case GFSM_SETPDUSMS:			//Sets SMS in Text mode
			case GFSM_SETTXTSMS:			//Sets SMS in PDU mode
				#ifdef DEBUGFSM
				if(GsmFsmState == GFSM_SETTXTSMS) {
					DEBUGENTER(GFSM_SETTXTSMS);
				} else {
					DEBUGENTER(GFSM_SETPDUSMS);
				}
				#endif
				//Command Mode
//				SystemStatus |= SS_RESPONSE;//Next time enter Response mode of this state
				InChr = '0';				//Set value for PDU mode
				if(GsmFsmState == GFSM_SETTXTSMS) {
					InChr = '1';			//Or set value for Text mode
				}
				UartParams->readTimeout = 1000000/Clock_tickPeriod;
				//The command needs to be sent in three parts, one is the main command
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CMGF], 8);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, &InChr, 1);//The second is the parameter
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CMGF] +8, 2);//'\r\n'
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 25);
				//Response mode
				/*In normal operation only an OK can be the answer of AT+CMGF=x command.
				There is also the possibility of +CMTI message in the queue. In that case we
				can ignore it and still wait for OK answer of AT+CMGF command. The format of
				a received CMTI is "+CMTI: <mem>,<index>" where <mem> is the memory used in
				double quotes and the <index> is the number of memory cell used for the SMS*/
				if(InChars >= 6) {			//OK response? do the rest
					SystemStatus &= ~SS_RESPONSE;	//Going to enter command mode
					if(GsmFsmState == GFSM_SETPDUSMS) {
						GsmFsmState = GFSM_SMSLIST;	//Next state is to check stored SMSs (PDU
													// Mode)
						/*The rest of the code that runs in this state should not break. In
						this way the system enters GFSM_SMSLIST state at once, as this is the
						one that follows. NEWDATA is still active, to force entering the
						following state.*/
					} else {
						GsmFsmState = GFSM_READSMS;	//Next state is to read SMSs (Text Mode)
						SystemStatus |= SS_NOEXIT;	//Enter the new state at once
						DEBUGEXIT(1);
						break;						//Exit this state
					}
				} else {					//Erroneous response =>
					FSMEnterPowerOff();		//Need to enter Power Off state
					DEBUGEXIT(255);
					break;
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_SMSLIST:				//Lists all messages
				DEBUGENTER(GFSM_SMSLIST);
				//Failed to decode the message's data
				if((SMSStatus & SMS_FAIL) != 0) {
					//Found erroneous response earlier? Yes =>
					if((SMSStatus & SMS_NEEDDELAY) == 0) {//First time we found FAIL flag?
						//Set a proper timeout to ensure AT+CMGL command ends
						UartParams->readTimeout = 300000/Clock_tickPeriod;
						do {
							tmpLen = UART_read(GSMUartHandle, GsmInBuffer, 1);
						} while(tmpLen > 0);
//						SMSStatus |= SMS_NEEDDELAY;	//NEEDDELAY is served
					}
					//Clear the Fail status of the SMS sys
					SMSStatus &= ~(SMS_FAIL | SMS_NEEDDELAY);
					//Prepare SystemStatus for a repeat of command
//					SystemStatus &= ~(SS_NEWDATA | SS_SECLINE | SS_RESPONSE | SS_GPRSDELAY);
					SystemStatus &= ~(SS_RESPONSE | SS_SECLINE);
					if(++GsmRepeatCnt >= CMGL_MAXREPEAT) {//Try once more. Out of retries?
						GsmRepeatCnt = 0;	//Clear the repeat counter
						//The GSM channel is very noisy...
						FSMEnterPowerOff();
						DEBUGEXIT(254);
						break;				//Exit the state
					}
				}
				//Command Mode
				if((SystemStatus & SS_RESPONSE) == 0) {
					SMSMemLength = 0;		//Array of currently stored SMS data is empty
					SMSMemIndex = 0;		//Beginning of SMSMem array
					/*Reset SMS status. Keep Short delay bit and flash parameters storage
					flags untouched.*/
					SMSStatus &= (SMS_SHORTDELAY | SMS_FLASHMASK);
					/*After sending the command to the module, we enter response mode,
					expecting the first line of its response.*/
//					SystemStatus &= ~(SS_NEWDATA | SS_SECLINE);
					SystemStatus &= ~SS_SECLINE;
					SystemStatus |= SS_RESPONSE;
					UartParams->readTimeout = 2000000/Clock_tickPeriod;
					/*Issue a list SMSs command to GSM Module, Do not change the status of
					the SMS to "READ".*/
					InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CMGL], 13);
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;					//Exit the state
					}
					InChars = UART_read(GSMUartHandle, GsmInBuffer, GSMINSIZE);
					if(InChars == 0) {		//Timeout => Communications Loss...
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;
					}
				}
				//Response mode
				if((SystemStatus & (SS_RESPONSE | SS_SECLINE)) == SS_RESPONSE) {
					//First line of response +CMGL (or +CMTI) or OK
					UartParams->readTimeout = 500000/Clock_tickPeriod;
					/*In the first line of AT+CMGL response we expect either OK, when this is
					the final line, or +CMGL: <index>,<stat>,... Also the very first line of
					the response, may contain +CMTI: <mem>,<index> in case a new message just
					arrived from the SCMC. This line can be ignored, as AT+CMGL will be
					executed just after that and any received message, announced using +CMTI
					will be included in the list.*/
					//Most of the flags should be reset when entering this state mode
					SMSStatus &= (SMS_SHORTDELAY | SMS_FLASHMASK | SMS_MORE);
					/*First lets check for the OK answer of GSM module. This is the last line
					of this response. We only check for the first character after the CRLF
					prefix of the line.*/
					if(GsmInBuffer[2] == 'O') {
						/*If we receive OK, we have to proceed to the next state. If there
						are any messages stored in the SMS array, we have to check them, so
						Set Text Message follows. If the array of received messages is empty
						we enter Wait For New SMS state. The new state must be entered in
						command mode, so the first thing we do is to set SystemStatus*/
						SystemStatus &= ~(SS_RESPONSE | SS_SECLINE);
						stTime = Clock_getTicks();
						InChars = 0;
						GsmFsmState = GFSM_WAITNEXT;		//Prepare to wait for a new SMS
						if(SMSMemLength > 0) {
							GsmFsmState = GFSM_SETTXTSMS;	//Must get the messages...
							SystemStatus |= SS_NOEXIT;
							DEBUGEXIT(1);
							break;			//Exit this state to enter the new one at once
						}
						/*The rest of the code that runs in this state should not break. In
						this way the system enters the following state at once, whichever it
						is. NEWDATA is still active, to force entering the following state.*/
					} else if(GsmInBuffer[6] =='I') {
						/*+CMTI response. Must ignore it, remove this line from buffer,
						calculate if there is a complete line in buffer and if not, issue
						another UART_read to complete the line. That line must be manipulated
						as being the first line of the response.*/
						TestVal = FindCRLF(&GsmInBuffer[7], InChars -7);
						if(TestVal <= 0) {	//Hmmm... Something is wrong...
							FSMEnterPowerOff();
							DEBUGEXIT(253);
							break;
						} else {
							/*Because the searching started at index 7, the TestVal value
							counts from that point. So the real index of the first CRLF is at
							TestVal +7. We need to skip two more characters, as they are the
							terminating CRLF of the line just scanned. That means TestVal
							must be increased by 9 to contain the correct index in Gsm Buffer
							that the new line starts.*/
							TestVal += 9;
							//Now copy the rest of the line to the beginning of the buffer
							for(tmpLen = 0; tmpLen < (InChars - TestVal); tmpLen++) {
								GsmInBuffer[tmpLen] = GsmInBuffer[TestVal + tmpLen];
							}
							/*Update InChars to contain the real number of characters in the
							buffer. There is also the possibility of not receiving the last
							terminating character. In that case TestVal is greater than
							InChars. We can read the missing character and proceed safely.*/
							InChars = tmpLen;
							TestVal = FindCRLF(&GsmInBuffer[2], InChars -2);
							if(TestVal < 0) {//Erroneous characters found...
								FSMEnterPowerOff();
								DEBUGEXIT(252);
								break;
							} else if((TestVal == 0) || (TestVal >= (InChars -3))) {
								//Incomplete line in buffer. Read the next part to fill it in
								InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars],
									GSMINSIZE -InChars);
								//Now need to re-enter the state at Response mode, first line
								SystemStatus |= SS_NOEXIT;	//Execute the next state at once
								SMSStatus |= SMS_MORE;		//Flag there are more messages
															// stored in SIM card
								DEBUGEXIT(128);
								break;						//Force re-entrance of this state
							}
						}
					} else if(GsmInBuffer[6] == 'L') {
						/*+CMGL response. Must store the index number, and ensure we have two
						hex digits from the new line. If not, we just read the missing digits
						and pass control to the Second Line Response code.*/
						SystemStatus |= SS_SECLINE;
						/*If there is space in SMS array get the index of the message and
						store it in SMSMem's current element.*/
						if(SMSMemLength < SMSMEMSIZE) {
							SMSMem[SMSMemLength].index = atoi(&GsmInBuffer[9]);
						} else {			//else,
							//Flag there are more messages stored in SIM card
							SMSStatus |= SMS_MORE;
						}
						//Lets find the end of this line, by searching for CRLF characters
						TestVal = FindCRLF(&GsmInBuffer[7], InChars -7);
						if(TestVal <= 0) {	//Erroneous characters found, or not found at all
							/*The reading was big enough to hold a whole line of the response
							including the terminating CRLF. If not even one of CR or LF
							characters were found, then probably the response is corrupted
							and the UART synchronization is lost. So, power off.*/
							FSMEnterPowerOff();
							DEBUGEXIT(251);
							break;
						} else {
							TestVal += 9;	//For the same reason described in +CMTI block
							//Now copy the rest of the line to the beginning of the buffer
							for(tmpLen = 0; tmpLen < (InChars - TestVal); tmpLen++) {
								GsmInBuffer[tmpLen] = GsmInBuffer[TestVal + tmpLen];
							}
							/*Update InChars to contain the real number of characters in the
							buffer. There is also the possibility of not receiving the last
							terminating character. In that case TestVal is greater than
							InChars. We can read the missing character and proceed safely.*/
							InChars = tmpLen;
							if(InChars < 2){
								//Need two valid hex digits (No CRLF exists), so read more
								InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars],
									2 -InChars);
								//Now need to re-enter at Response mode, second line
								SystemStatus |= (SS_SECLINE | SS_NOEXIT);
								DEBUGEXIT(129);
								break;		//Force re-entrance of this state
							}
						}
					} else {				//Neither OK, CMTI nor CMGL response? => Unknown
						/*Garbage response. Need  to discard all data by waiting two seconds,
						clearing the input buffer and then recheck the list of incoming SMSs.
						This should happen at most 3 times. After that the system should
						power off the GSM module and repeat after the propper timeout.*/
						//Signal garbage data in serial stream
						SMSStatus |= SMS_FAIL;
						SystemStatus |= SS_NOEXIT;
						DEBUGEXIT(130);
						break;						//Exit state, to wait for the timeout
						/*NEWDATA flag is not reset, so there will be another entrance in the
						state at once. SMS_Fail code will take care of it.*/
					}
				}
				//Second line of response: PDU data
				/*In the second line of the response we expect PDU data. All the line should
				be comprised of hex digits. No other character is accepted. The SMS messages
				we can use are SMS-DELIVER, 7 or 8 bits character set, no compression and no
				special data in the message that require a User Data Header*/
				if((SystemStatus & (SS_RESPONSE | SS_SECLINE)) == (SS_RESPONSE | SS_SECLINE)){
					//First, skip the heading part of PDU
					//Convert the first two Hex digits into a number (length of heading)
					tmpLen = Hex2Int(GsmInBuffer, 2);
					if(tmpLen == -1) {		//Again, garbage characters found =>
						//Signal garbage data in serial stream
						SMSStatus |= SMS_FAIL;
						SystemStatus |= SS_NOEXIT;
						DEBUGEXIT(131);
						break;				//Exit state, to wait for the timeout
						/*There will be another entrance in the state at once. SMS_Fail code
						will take care of it.*/
					}
					//Now copy the rest of the line to the beginning of the buffer
					for(TestVal = 0; TestVal < (InChars -2); TestVal++) {
						GsmInBuffer[TestVal] = GsmInBuffer[TestVal +2];
					}
					InChars = TestVal;		//Moved the line two digits left
					TestVal = tmpLen *2 +4;
					if(TestVal > InChars) {	//Need more characters? => Read them
						InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars],
							TestVal - InChars);
						if(InChars != TestVal) {	//Hmmm... Response was truncated!
							SMSStatus |= SMS_FAIL;	//Signal garbage data in serial stream
							SystemStatus |= SS_NOEXIT;
							DEBUGEXIT(132);
							break;			//Exit state, to wait for the timeout
						}
					}
					//Now need to parse the message type and see if it is acceptable
					tmpLen = Hex2Int(&GsmInBuffer[TestVal -4], 2);//Convert message type octet
					/*The message type octet is a bit field that presents what is going on
					with the message. Bits 0,1 must be both zeroed as they describe an
					SMS_DELIVER message. Bit 2 flags if there are more messages to receive.
					For us is "do not care!". Bit 3 is the loop prevention bit and is also a
					"do not care" for us. Bit 4 is not used, Bit 5 is the flag that announces
					if a deliver report is requested by the sender. We do not care, again.
					Bit 6 is the flag that there is also a header in user data field. Since we
					do not parse extra SMS features (like multispanned SMS) we must be sure
					there is no header in the user data, so it must be 0. Bit 7 is the Reply
					Path Present bit. We do not need a reply path since we do not parse extra
					features of an SMS*/
					if(tmpLen == -1) {		//Again, garbage characters found =>
						SMSStatus |= SMS_FAIL;//Signal garbage data in serial stream
						SystemStatus |= SS_NOEXIT;
						DEBUGEXIT(133);
						break;				//Exit state, to wait for the timeout
						/*There will be another entrance in the state at once. SMS_Fail code
						will take care of it.*/
					}
					tmpLen &= 0xD3;			//Filter only the the needed bits
					if(tmpLen != 0) {		//Not porpperly formated SMS => Discard
						if(SMSMemLength < SMSMEMSIZE) {
							SMSMem[SMSMemLength++].length = 0;
						}
						//Going to discard the rest of the message's line
					} else {
						//Get Originating address length. We need to discard it
						tmpLen = Hex2Int(&GsmInBuffer[TestVal -2], 2);
						/*The Originating Address Length is in semi-octets = hex characters
						needed. But, if the characters number is odd, then we need to read
						another one to complete the last octet.*/
						if(tmpLen == -1) {	//Again, garbage characters found =>
							SMSStatus |= SMS_FAIL;//Signal garbage data in serial stream
							SystemStatus |= SS_NOEXIT;
							DEBUGEXIT(134);
							break;			//Exit state, to wait for the timeout
							/*There will be another entrance in the state at once. SMS_Fail
							code will take care of it.*/
						}
						//If the length is an odd number then increment by one => even
						tmpLen = tmpLen + (tmpLen & 1);
						//Now copy the rest of the line to the beginning of the buffer
						for(TestPar = 0; TestPar < (InChars -TestVal); TestPar++) {
							GsmInBuffer[TestPar] = GsmInBuffer[TestVal +TestPar];
						}
						InChars = TestPar;
						//Do we have all these characters in buffer? No => Read them
						if(InChars < tmpLen +22) {
							/*Read all of the Originating Address field, including the type of
							address, PID, DCS, SMSC timestamp, and UD Length.*/
							InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars],
								tmpLen +22 -InChars);
							if(InChars != (tmpLen +22)) {
								/*Hmmm... Response was truncated! Signal garbage data in
								serial stream.*/
								SMSStatus |= SMS_FAIL;
								SystemStatus |= SS_NOEXIT;
								DEBUGEXIT(135);
								break;		//Exit state, to wait for the timeout
							}
						}
						//PID parsing follows. Only some PIDs are acceptable
						//Get PID number, filter it
						TestVal = Hex2Int(&GsmInBuffer[tmpLen +2], 2) & 0xBF;
						if(TestVal == -1) {	//Again, garbage characters found =>
							//Signal garbage data in serial stream
							SMSStatus |= SMS_FAIL;
							SystemStatus |= SS_NOEXIT;
							DEBUGEXIT(136);
							break;			//Exit state, to wait for the timeout
							/*There will be another entrance in the state at once. SMS_Fail
							code will take care of it*/
						} else if(TestVal != 0) {
							//Not simple SMS? => Discard it
							if(SMSMemLength < SMSMEMSIZE) {
								SMSMem[SMSMemLength++].length = 0;
							}
							//Going to discard the rest of the message's line
						} else {
							//Parse DCS value. Must discard messages with unacceptable DCS
							//Get DCS value
							TestVal = Hex2Int(&GsmInBuffer[tmpLen +4], 2);
							if(TestVal == -1) {
								/*Again, garbage characters found => Signal garbage data in
								serial stream.*/
								SMSStatus |= SMS_FAIL;
								SystemStatus |= SS_NOEXIT;
								DEBUGEXIT(137);
								break;		//Exit state, to wait for the timeout
								/*There will be another entrance in the state at once.
								SMS_Fail code will take care of it.*/
							}
							if((TestVal & 0xE3) != 0) {
								//Not correct type of message? => Discard it
								if(SMSMemLength < SMSMEMSIZE) {
									SMSMem[SMSMemLength++].length = 0;
								}
								//Going to discard the rest of the message
							} else {
								//Filter only character type field
								TestVal &= 0x0C;
								if(TestVal == 0x0C) {
									/*Undefined/Reserved? => ETSI TS 123 038 directs that
									Reserved should be treated as GSM 7 Bit default
									alphabet.*/
									TestVal = 0;
								}
								if((TestVal & (1<<2)) != 0) {
									//GSM 8 Bit alphabet? => Flag it
									SMSStatus |= SMS_8BIT;
								}
								if((TestVal & (1<<3)) != 0) {
									//UCS2 16 Bit alphabet? => Flag it
									SMSStatus |= SMS_UCS2;
									if(SMSMemLength < SMSMEMSIZE) {
										//We still do not decode UCS2
										SMSMem[SMSMemLength++].length = 0;
									}
								//Going to discard the rest of the message
								} else {
									/*Finally, get the User Data Length; it is the length of
									the SMS in characters*/
									TestVal = Hex2Int(&GsmInBuffer[tmpLen+ 20], 2);
									if(tmpLen == -1) {
										/*Again, garbage characters found => Signal garbage
										data in serial stream.*/
										SMSStatus |= SMS_FAIL;
										SystemStatus |= SS_NOEXIT;
										DEBUGEXIT(138);
										break;
										/*Exit state, to wait for the timeout. There will be
										another entrance in the state at once. SMS_Fail code
										will take care of it*/
									}
									/*SMS length needed in number of characters found. Store
									it in the array of messages to be parsed.*/
									SMSMem[SMSMemLength++].length = TestVal;
									//Going to discard the rest of the message
								}
							}
						}
					}
					//Time to discard the rest of this line. Find first occurence of CR or LF
					TestVal = FindCRLF(GsmInBuffer, InChars);
					if(TestVal < 0) {		//Error => Signal garbage data in serial stream
						SMSStatus |= SMS_FAIL;
						SystemStatus |= SS_NOEXIT;
						DEBUGEXIT(139);
						break;				//Exit state, to wait for the timeout
						/*There will be another entrance in the state at once. SMS_Fail code
						will take care of it.*/
					} else if(TestVal == 0) {
						do {				//Read data until CR/LF character
							tmpLen = UART_read(GSMUartHandle, GsmInBuffer, 1);
						} while((GsmInBuffer[0] != '\r') && (GsmInBuffer[0] != '\n') &&
							(tmpLen > 0));
						if(tmpLen == 0){	//Signal garbage data in serial stream
							SMSStatus |= SMS_FAIL;
							SystemStatus |= SS_NOEXIT;
							DEBUGEXIT(140);
							break;			//Exit state, to wait for the timeout
							/*There will be another entrance in the state at once. SMS_Fail
							code will take care of it.*/
						}
						//Only the first character fo CRLF is read. Read the second one
						tmpLen = UART_read(GSMUartHandle, GsmInBuffer, 1);
//						//Clear NEWDATA flag and expect first line of response
//						SystemStatus &= ~(SS_NEWDATA | SS_SECLINE);
						//The first line of the next response block is expected
						SystemStatus &= ~SS_SECLINE;
						SystemStatus |= SS_NOEXIT;
//						UartParams->readTimeout = 100000/Clock_tickPeriod;
						InChars = UART_read(GSMUartHandle, GsmInBuffer, GSMINSIZE);
						DEBUGEXIT(2);
						break;				//Exit state, to wait for the next line
					} else {
						/*We have to skip the terminating CRLF, so we add two more characters.
						If TestVal is greater than the number of characters in the buffer then
						the CRLF is not complete. So read one more character.*/
						if(TestVal > (InChars -2)) {
							InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars], 1);
						}
						TestVal += 2;		//Skip the terminating CRLF of this line
						//Lets copy the rest of the characters in buffer to its beginning
						for(tmpLen = 0; tmpLen < (InChars -TestVal); tmpLen++){
							GsmInBuffer[tmpLen] = GsmInBuffer[TestVal +tmpLen];
						}
						InChars = tmpLen;	//Now buffer contains tmpLen characters
						//Do we have a complete line in buffer?
						TestVal = FindCRLF(&GsmInBuffer[2], InChars -2);
						if(TestVal < 0) {	//Erroneous characters found => Fail
							//Signal garbage data in serial stream
							SMSStatus |= SMS_FAIL;
							SystemStatus |= SS_NOEXIT;
							DEBUGEXIT(141);
							break;			//Exit state, to wait for the timeout
							/*There will be another entrance in the state at once. SMS_Fail
							code will take care of it.*/
						} else if(TestVal == 0){
							//Fill the rest of the line to be parsed...
							InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars],
								GSMINSIZE -InChars);
						}
//						//Clear NEWDATA flag and expect first line of response
//						SystemStatus &= ~(SS_NEWDATA | SS_SECLINE);
						SystemStatus &= ~SS_SECLINE;
						SystemStatus |= SS_NOEXIT;
						DEBUGEXIT(14);
						break;				//Exit state, to wait for the next line
					}
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_WAITNEXT:				//Waits at most 40 seconds until receiving a new
											// SMS message
				DEBUGENTER(GFSM_WAITNEXT);
				//Command Mode
//				SystemStatus |= SS_RESPONSE;//Next time enter response mode
				/*There was no error on SMS reception part. No need to repeat it on next FSM
				itteration.*/
				ClkEvent &= ~(EV_SMSRECEIVE);
				nowTime = Clock_getTicks() - stTime;
				if((SMSStatus & SMS_SHORTDELAY) != 0) {
					//Short delay of 5 seconds
					endTime = 5000000/Clock_tickPeriod;
				} else {
					//Long delay of 30 seconds
					endTime = 30000000/Clock_tickPeriod;
					//Next time enter short delay cycle
					SMSStatus |= SMS_SHORTDELAY;
				}
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
				if(nowTime < endTime) {		//Need more time to wait?
					//Then prepare to fill the time remaining
					UartParams->readTimeout = endTime -nowTime;
					InChars = UART_read(GSMUartHandle, GsmInBuffer, GSMINSIZE);
				}
				//Response Mode
				if((InChars == 0) || (nowTime >= endTime)) {
//					//Clear the timeout flag. Next state will be entered in command mode
//					SystemStatus &= ~(SS_GPRSDELAY | SS_RESPONSE);
					StoreParams(SMSStatus);	//Store the received parameters (if any) in flash
					/*Going to attach to GPRS service and start communication to server
					without exiting the FSM loop, if it is needed (indicated by the event that
					started the FSM running.*/
					if((SystemStatus & (SS_HTTPSETS | SS_GSMTYPEREQ)) != 0) {
						GsmFsmState = GFSM_ATTACH;
//						//Simulate a new event to enter the new state at once
//						SystemStatus |= (SS_NEWDATA | SS_NOEXIT);
						SystemStatus |= SS_NOEXIT;
						DEBUGEXIT(0);
						break;				//Exit this state
					} else {
						//No need to perform a GPRS communication so power off
						GsmFsmState = GFSM_POWEROFF;
						SystemStatus |= (SS_NOEXIT | SS_POWEROFF | SS_NEWDATA);
						SystemStatus &= ~SS_GPRSDELAY;
						break;
					}
				}
				if(InChars < 20) {			//Lets ensure we have read all the received data
					//Just in case the timeout expired during reception of characters, read
					UartParams->readTimeout = (100000/Clock_tickPeriod);
					InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars],
						GSMINSIZE -InChars);
				}
				/*We may have received many lines of data. We must check every one of them to
				see if there is any +CMTI that announces the reception of a new SMS.*/
				tmpLen = 2;
				TestVal = 1;
				TestPar = 1;
				while((tmpLen < InChars) && (TestVal != 0) && (TestPar != 0)){
					//Do we have a "+CMTI: <mem>,<index>" response from GSM?
					TestVal = strncmp(&GsmInBuffer[tmpLen], InSMSInd, 7);
					TestPar = FindCRLF(&GsmInBuffer[tmpLen], InChars -tmpLen);
					TestPar += SkipCRLF(&GsmInBuffer[tmpLen +TestPar],
						InChars -(tmpLen + TestPar));
					tmpLen += TestPar;
				}
//				//Next state is entered in Command mode at once, without exiting FSM
//				SystemStatus &= ~SS_RESPONSE;
				SystemStatus |= SS_NOEXIT;
				//If there is any new message we must parse it
				if(TestVal == 0) {			//+CMTI found
					//Will retry list of messages
					GsmFsmState = GFSM_SMSLIST;
					DEBUGEXIT(1);
					break;
				}
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
				/*We have received new lines of response but not a +CMTI, so wait more until
				the timeout expires. To achieve that, just reenter the same state.*/
				DEBUGEXIT(2);
				break;						//Exit state

			//--------------------------------------
			case GFSM_READSMS:
				DEBUGENTER(GFSM_READSMS);
				//Command Mode
				if((SystemStatus & SS_RESPONSE) == 0) {
					if(SMSMemIndex < SMSMemLength) {
						if(SMSMem[SMSMemIndex].length == 0) {
							/*Do we have to ignore this message? => Yes, Must go to Delete
							Message state at once, without exiting the state machine loop.*/
							GsmFsmState = GFSM_DELSMS;
							SystemStatus |= SS_NOEXIT;
							/*By leaving NEWDATA flag active, the FSM enters the new state at
							once. The code that runs from here on must no break for the
							DELSMS state to be executed without even looping in the FSM, as
							DELSMS state follows this one.*/
						} else {			//else, we must read and parse it
							SystemStatus |= SS_RESPONSE;	//Next time enter response mode
//							SystemStatus &= ~(SS_NEWDATA | SS_SECLINE);
							SystemStatus &= ~SS_SECLINE;
							/*Issue a Read SMS command to the GSM module. The command will be
							sent in two parts, one is the main command string and the second
							one is the number of the index and the terminating CRLF.*/
							InChars = UART_write(GSMUartHandle,
								GSM_Commands[GSM_CMD_CMGR], 8);
							if(InChars == UART_ERROR) {	//If there was a timeout during write,
								// the return value is UART_ERROR. In that case we have to
								// gracefully exit.
								// => Error. Need to power off
								FSMEnterPowerOff();
								DEBUGEXIT(255);
								break;					//Exit the state
							}
							/*At the end of the target buffer, add a CRLF and a terminating
							character. The last one is not necessary...*/
							GsmInBuffer[GSMINSIZE -3] = '\r';
							GsmInBuffer[GSMINSIZE -2] = '\n';
							GsmInBuffer[GSMINSIZE -1] = '\0';
							TestVal = Int2Ascii(SMSMem[SMSMemIndex].index, GsmInBuffer,
								GSMINSIZE -3);
							UartParams->readTimeout = 1000000/Clock_tickPeriod;
							InChars = UART_write(GSMUartHandle,
								&GsmInBuffer[GSMINSIZE -3 -TestVal], TestVal +2);
							if(InChars == UART_ERROR) {	//If there was a timeout during write,
								// the return value is UART_ERROR. In that case we have to
								// gracefully exit.
								// => Error. Need to power off
								FSMEnterPowerOff();
								DEBUGEXIT(255);
								break;					//Exit the state
							}
							InChars = UART_read(GSMUartHandle, GsmInBuffer, GSMINSIZE);
							if(InChars == 0) {
								//Timeout occurred: Power off the module and exit
								FSMEnterPowerOff();
								DEBUGEXIT(255);
								break;
							}
							/*At this point the state is transfered to Response mode, first
							line.*/
						}
					} else {				//Did we finish parsing all the messages found?
						GsmFsmState = GFSM_DELALLSMS;	//Delete all read SMS
						/*The code that runs to the end of this state must not break. In this
						way the FSM enters DELALLSMS at once, without looping, as DELALLSMS
						state's code follows this one.*/
					}
				}
				if((SystemStatus & SS_RESPONSE) != 0) {
					//Response Mode
					if((SystemStatus & SS_SECLINE) == 0) {
						/*First line response: Can be OK, +CMTI: <mem>,<index>,
						+CMGR: <stat>,<oa>, +CME ERROR:<err> or garbage characters.*/
						//First lets check for "\r\nOK\r\n" response
						if(GsmInBuffer[2] == 'O') {
							/*Received OK => This message is done. Proceed to the next one
							(if exists) or exit.*/
							SMSMemIndex++;	//Next message will be parsed
							//Next state is going to be entered in Command mode
							SystemStatus &= ~SS_RESPONSE;
							if(SMSMemIndex >= SMSMemLength) {
								/*Run out of messages? => going to delete all SMS read from
								memory in Command mode.*/
								GsmFsmState = GFSM_DELALLSMS;
								SystemStatus &= ~SS_RESPONSE;
								/*The code that runs to the end of this state must not break.
								In this way the FSM enters DELALLSMS state at once, without
								looping, as DELALLSMS's code follows this one.*/
							} else {
								/*More messages => Need to run READSMS again without exiting
								the FSM loop*/
								SystemStatus |= SS_NOEXIT;
								DEBUGEXIT(1);
								break;		//Stop this state to reenter it at once in
											// Command mode
							}
						} else if(GsmInBuffer[5] == 'T') {
							/*+CMTI response => Remove this line from buffer (Ignore it),
							calculate if there is a complete line in buffer and if not, issue
							another UART read command to fill in the buffer with the rest of
							the missing data. The newly received line must be handled as if
							it is the first line of the response.*/
							TestVal = FindCRLF(&GsmInBuffer[7], InChars -7);
							if(TestVal <= 0) {	//Hmmm... Something is wrong...
								FSMEnterPowerOff();
								DEBUGEXIT(254);
								break;
							}
							/*Because the searching started at index 7, the TestVal value
							counts from that point. So the real index of the first CRLF is at
							TestVal +7. We need to skip two more characters, as they are the
							terminating CRLF of the line just scanned. That means TestVal
							must be increased by 9 to contain the correct index in GSM Buffer
							that the new line starts.*/
							TestVal += 9;
							//Now copy the rest of the line to the beginning of the buffer
							for(tmpLen = 0; tmpLen < (InChars - TestVal); tmpLen++) {
								GsmInBuffer[tmpLen] = GsmInBuffer[TestVal + tmpLen];
							}
							/*Update InChars to contain the real number of characters in the
							buffer. There is also the possibility of not received the last
							terminating character. In that case TestVal is greater than
							InChars. We can read the missing character and proceed safely.*/
							InChars = tmpLen;
							TestVal = FindCRLF(&GsmInBuffer[2], InChars -2);
							if(TestVal < 0) {//Erroneous characters found...
								FSMEnterPowerOff();
								DEBUGEXIT(253);
								break;
							} else if((TestVal == 0) || (TestVal >= (InChars -3))) {
								//Incomplete line in buffer => Read the next part to fill
								// it in
								InChars += UART_read(GSMUartHandle,
									&GsmInBuffer[InChars], GSMINSIZE -InChars);
								//Now need to re-enter the state at Response mode, first line
								SystemStatus |= SS_NOEXIT;	//Execute the next state at once
								SMSStatus |= SMS_MORE;		//Flag there are more messages
															// stored in SIM card
								DEBUGEXIT(2);
								break;						//Force re-entrance of this state
							}
						} else if(GsmInBuffer[5] == 'G') {
							/*+CMGR response => Need to remove this line from buffer, as in
							+CMTI response, ensure that we have the full message in buffer
							and proceed to the Second Line processing, which is the real
							message.*/
							TestVal = FindCRLF(&GsmInBuffer[7], InChars -7);
							if(TestVal <= 0) {	//Hmmm... Something is wrong...
								FSMEnterPowerOff();
								DEBUGEXIT(252);
								break;
							}
							/*Because the searching started at index 7, the TestVal value
							counts from that point. So the real index of the first CRLF is at
							TestVal +7. We need to skip two more characters, as they are the
							terminating CRLF of the line just scanned. That means TestVal
							must be increased by 9 to contain the correct index in GSM Buffer
							that the new line starts.*/
							TestVal += 9;
							//Now copy the rest of the line to the beginning of the buffer
							for(tmpLen = 0; tmpLen < (InChars - TestVal); tmpLen++) {
								GsmInBuffer[tmpLen] = GsmInBuffer[TestVal + tmpLen];
							}
							/*Update InChars to contain the real number of characters in the
							buffer. There is also the possibility of not received the last
							terminating character. In that case TestVal is greater than
							InChars. We can read the missing character and proceed safely.*/
							InChars = tmpLen;
							/*The length of the SMS in characters is stored in variable
							SMSMem.length. It is a nice idea for the SMS to be present in
							the buffer, all characters of it. So lets ensure this is
							true, including the terminating CRLF (2 characters more).*/
							tmpLen = SMSMem[SMSMemIndex].length +2;
							if(InChars < tmpLen) {
								InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars],
									tmpLen -InChars);
							}
							//Clear found flags
//							SystemStatus &= ~(SS_NEWDATA | SS_ALLNOTFOUND | SS_FOUNDALL);
							SystemStatus &= ~(SS_ALLNOTFOUND | SS_FOUNDALL);
							//Next time the SMS's text will be fetched
							SystemStatus |= SS_SECLINE;
							/*Must set the number of characters to be read for the SMS. The
							counting should start from the beginning of the SMS, meaning just
							after what characters were extracted from GsmInBuffer in the
							previous part. During that period there may be some characters
							already read from the GSM module, that belong to that counting.
							Those characters will be received when UART_read executes because
							of the hardware flow control.*/
//							GsmLimitChrs = SMSMem[SMSMemIndex].length;// -GsmInLength;
							//Clear the limit flags
//							GSMSerialStatus &= ~(GSMS_LIMITED | GSMS_LIMITREAD |
//								GSMS_LIMITACK);
//							GSMSerialStatus |= GSMS_COUNTER;//Count the incoming characters
							ParamsFound = 0;				//No parameters found, yet
							/*At this point we have the full SMS in buffer and must
							proceed to the Second Line part of response, to parse it. No
							need to break, as the Second Line Parsing code is entered at
							once.*/
						} else {			//+CME ERROR response or garbage characters... =>
							FSMEnterPowerOff();
							DEBUGEXIT(251);
							break;
						}
					}
					if((SystemStatus & (SS_RESPONSE | SS_SECLINE)) ==
						(SS_RESPONSE | SS_SECLINE)) {
						/*Second and subsequent lines for length specified in SMSMem array are
						the pure message body to be parsed.*/
//						SystemStatus &= ~SS_NEWDATA;	//Flag that the new data is processed
						//Another FSM. Start from skipping first non letter characters
						SearchState = SRCH_SKIPWHITE;
						CharCnt = 0;		//Counter of characters in input stream
						while((CharCnt < tmpLen) &&
							((SystemStatus & (SS_ALLNOTFOUND | SS_FOUNDALL)) == 0)) {
							switch (SearchState) {		//Parameter searching state machine
							case SRCH_SKIPWHITE:
								//Skip white characters (not letters or digits)
								for(TestPar = 0; TestPar < SMSPARAMSNO; TestPar++) {
									//Must reset the parameter searching table
									ParamSearch[TestPar].offset = 0;		//Reset offset
									ParamSearch[TestPar].status = PS_SCAN;	//Reset Status
								}
								ParamsFound = 0;//Number of parameters found
								NFCount = 0;	//Number of parameters not found
								do {			//Have to skip all non letter characters
									InChr = GsmInBuffer[CharCnt++];
								} while(((InChr < '0') || ((InChr > '9') && (InChr < 'A')) ||
									((InChr > 'Z') && (InChr < 'a')) || (InChr > 'z')) &&
										(CharCnt < tmpLen));
								CharCnt--;		//Must revert the last CharCnt increment
								//Clear! Must search for parameter existence
								SearchState = SRCH_SEARCH;
								break;

							case SRCH_SEARCH:
								//Search through all parameters
								for(CurrParam = 0; CurrParam < SMSPARAMSNO; CurrParam++){
									if((GsmInBuffer[CharCnt] >= 'a') &&
										(GsmInBuffer[CharCnt] <= 'z')) {
										//Convert small letter to capital one
										GsmInBuffer[CharCnt] &= 0xDF;
									}
									//Do we have a match?
									if(ParamSearch[CurrParam].status == PS_SCAN) {
										InChr = SMSParamArr[CurrParam].str[
													ParamSearch[CurrParam].offset];
										if(InChr == '\0') {	//Termination of the parameter?
											/*Have to know if this is the whole word. After
											the parameter an '=' or ':' sign is expected.*/
											InChr = GsmInBuffer[CharCnt];
											//Going to skip all subsequent space characters
											while(InChr == ' ') {
												CharCnt++;		//Point to next character
												InChr = GsmInBuffer[CharCnt];//And read it
											}
											if((InChr != ':') && (InChr != '=')) {
												//Not default assignment char?
												//Flag the current parameter as not found
												ParamSearch[CurrParam].status = PS_NOTFOUND;
												ParamSearch[CurrParam].offset = 0;
												NFCount++;
											} else {
												/*Flag the parameter as found and prepare to
												execute its associated function.*/
												ParamSearch[CurrParam].status = PS_FOUND;
												ParamsFound++;	//One more parameter found
												//Going to exec its function
												SearchState = SRCH_EXEC;
												//Exit for loop. We found the needed parameter
												break;
											}
										} else if(GsmInBuffer[CharCnt] != InChr) {
											//Parameter is not found
											ParamSearch[CurrParam].status = PS_NOTFOUND;
											ParamSearch[CurrParam].offset = 0;
											NFCount++;
											//One more parameter is flagged as not found
										} else {
											/*We have a match in characters. Lets try the next
											character in buffer.*/
											ParamSearch[CurrParam].offset++;
										}
										if((NFCount + ParamsFound) >= SMSPARAMSNO) {
											//All parameters tested?
											//Must give it a shot at the next word
											SearchState = SRCH_SKIPLETTER;
											//No need to run the for loop
											break;
										}
									}
								}
								CharCnt++;	//Next character
								break;

							case SRCH_EXEC:	//Parameter's function execution
								/*Need to execute the parameter's associated function and
								count out the characters it used.*/
								CharCnt += SMSParamArr[CurrParam].func(CharCnt);
								//Skip all the remaining normal letters
								SearchState = SRCH_SKIPLETTER;
								/*The advancing to the next state in this state machine is
								made at once. No break command needed.*/

							case SRCH_SKIPLETTER:
								do {		//Skip all the following letters
									InChr = GsmInBuffer[CharCnt++];
								} while(((InChr >= '0') && (InChr <= '9')) ||
									((InChr >= 'A') && (InChr <= 'Z')) ||
									((InChr >= 'a') && (InChr <= 'z')));
								CharCnt--;	//Undo the last pointer advancing
								//Reset the state machine and skip the rest of the non-letter
								// characters
								SearchState = SRCH_SKIPWHITE;
								break;
							}
						}
						/*We must remove the message line we've just parsed and ensure there
						is a whole new line in the buffer (presumably "\r\nOK\r\n").*/
						for(TestVal = 0; TestVal < (InChars -tmpLen); TestVal++) {
							GsmInBuffer[TestVal] = GsmInBuffer[tmpLen + TestVal];
						}
						InChars = TestVal;
						if(InChars < 6) {
							/*Need at least 6 characters for the OK response, but we will ask
							for more*/
							InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars],
								GSMINSIZE -InChars);
						}
						SystemStatus &= ~SS_SECLINE;//Next time we will read a status line
//						//Fix flags
//						GSMSerialStatus &= ~(GSMS_LIMITREAD | GSMS_LIMITED | GSMS_LIMITACK);
						SystemStatus |= SS_NOEXIT;
						DEBUGEXIT(3);
						break;
					}
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_DELSMS:				//Going to delete current SMS
			case GFSM_DELALLSMS:			//Going to delete ALL Read SMS Messages
			#ifdef DEBUGFSM
			if(GsmFsmState == GFSM_DELALLSMS) {
				DEBUGENTER(GFSM_DELALLSMS);
			} else {
				DEBUGENTER(GFSM_DELSMS);
			}
			#endif
			if((SystemStatus & SS_RESPONSE) == 0) {
				//Command Mode
//				SystemStatus &= ~SS_NEWDATA;//Clear NEWDATA flag
				SystemStatus |= SS_RESPONSE;//Next time enter response mode
				UartParams->readTimeout = 2000000/Clock_tickPeriod;
				//Send the base command to delete message(s)
				InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CMGD], 8);
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				if(GsmFsmState == GFSM_DELSMS) {	//One SMS or All? One =>
					//Terminating character of the target index in ASCII
					GsmInBuffer[GSMINSIZE -1] = '\0';
					TestVal = Int2Ascii(SMSMem[SMSMemIndex].index, GsmInBuffer, GSMINSIZE -1);
					//Send the calculated string to GSM module
					InChars = UART_write(GSMUartHandle, &GsmInBuffer[GSMINSIZE -1 -TestVal],
						TestVal);
					if(InChars == UART_ERROR) {	//If there was a timeout during write, the
						// return value is UART_ERROR. In that case we have to gracefully
						// exit.
						// => Error. Need to power off
						FSMEnterPowerOff();
						DEBUGEXIT(255);
						break;					//Exit the state
					}
					InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CMGD] +11, 2);
				} else {
					//Need to delete all SMSs =>
					InChars = UART_write(GSMUartHandle, GSM_Commands[GSM_CMD_CMGD] +8, 5);
				}
				if(InChars == UART_ERROR) {	//If there was a timeout during write, the return
					// value is UART_ERROR. In that case we have to gracefully exit.
					// => Error. Need to power off
					FSMEnterPowerOff();
					DEBUGEXIT(255);
					break;					//Exit the state
				}
				InChars = UART_read(GSMUartHandle, GsmInBuffer, 19);
				if(InChars == 0) {			//Timeout? => Sync lost...
					FSMEnterPowerOff();		//Power off the module
					DEBUGEXIT(255);
					break;
				}
			}
			if((SystemStatus & SS_RESPONSE) != 0) {
				//Response Mode
				/*The incoming line can be OK on success, ERROR on failure, +CME ERROR: <err>
				in case of GSM Module's problem, or +CMTI which means a new incoming message
				appeared. Do not forget the possibility of incoming garbage characters.
				In case of OK, or ERROR we continue to read all new SMSs; ERROR actually
				should never happen. It may be a programming error.
				+CME ERROR: <err> can happen only if there is a programming error, which means
				we need to fix the possible bug. In that case it is a good idea to power off
				the GSM module. The same happens if we have garbage characters.
				+CMTI: <mem>,<index> on the other hand should be ignored, as the new SMS will
				appear in the list of SMS issued by the following state of the FSM.*/
				if((GsmInBuffer[2] == 'O') || (GsmInBuffer[2] == 'E')) {
					//Do we have OK or ERROR response?
					//Next state will be entered in Command mode
					SystemStatus &= ~SS_RESPONSE;
					if(GsmFsmState == GFSM_DELSMS) {
						//Proceed to reading the next message
						SMSMemIndex++;
						GsmFsmState = GFSM_READSMS;
					}
					if((GsmFsmState == GFSM_DELALLSMS) || (SMSMemIndex >= SMSMemLength)){
						//In case of Deleting All messages or No more messages to parse =>
						SMSMemLength = 0;	//Clear the SMS Memory table
						SMSMemIndex = 0;	//Point to the beginning of the SMS Memory table
						//Going to reread the SMS list
						GsmFsmState = GFSM_SETPDUSMS;
					}
					SystemStatus |= SS_NOEXIT;
					DEBUGEXIT(1);
					break;					//Exit current state
				} else if(GsmInBuffer[6] == 'I') {
					/*Do we have +CMTI: <mem>,<index> response? => Must ignore this line and
					discard it from the buffer. Then we must ensure there is a whole new line
					in the buffer, to be parsed as being the response of the command.*/
					TestVal = FindCRLF(&GsmInBuffer[7], InChars -7);
					if(TestVal <= 0) {		//Hmmm... Something is wrong...
						FSMEnterPowerOff();
						DEBUGEXIT(254);
						break;
					}
					/*Because the searching started at index 7, the TestVal value
					counts from that point. So the real index of the first CRLF is at
					TestVal +7. We need to skip two more characters, as they are the
					terminating CRLF of the line just scanned. That means TestVal
					must be increased by 9 to contain the correct index in GSM Buffer
					that the new line starts.*/
					TestVal += 9;
					//Now copy the rest of the line to the beginning of the buffer
					for(tmpLen = 0; tmpLen < (InChars - TestVal); tmpLen++) {
						GsmInBuffer[tmpLen] = GsmInBuffer[TestVal + tmpLen];
					}
					/*Update InChars to contain the real number of characters in the
					buffer. There is also the possibility of not received the last
					terminating character. In that case TestVal is greater than
					InChars. We can read the missing character and proceed safely.*/
					InChars = tmpLen;
					TestVal = FindCRLF(&GsmInBuffer[2], InChars -2);
					if(TestVal < 0) {//Erroneous characters found...
						FSMEnterPowerOff();
						DEBUGEXIT(253);
						break;
					} else if((TestVal == 0) || (TestVal >= (InChars -3))){
						//Incomplete line in buffer => Read the next part to fill
						// it in
						InChars += UART_read(GSMUartHandle, &GsmInBuffer[InChars],
							19 -InChars);
						//Now need to re-enter the state at Response mode, first line
						SystemStatus |= SS_NOEXIT;	//Execute the next state at once
						SMSStatus |= SMS_MORE;		//Flag there are more messages
													// stored in SIM card
						DEBUGEXIT(2);
						break;						//Force re-entrance of this state
					}
				} else {
					//Garbage characters or +CME ERROR: <err>?
					PIN_setOutputValue(ledPinHandle, GSM_ERRORLED, 1);
					/*Clear the GPRS Delay and  Timeout flags. The absence of SS_NEWDATA flags
					an error state.*/
					SystemStatus &= ~(SS_GPRSDELAY | SS_NEWDATA);
					/*Have to follow Power Off state at once (and enable it through POWEROFF
					flag).*/
					SystemStatus |= SS_POWEROFF;
					GsmFsmState = GFSM_POWEROFF;
					DEBUGEXIT(252);
					/*At this point there is no reason to exit using "break", as Power Off
					state follows.*/
				}
			}

			//--------------------------------------
			case GFSM_POWEROFF:				//Need to power off the module, so
				DEBUGENTER(GFSM_POWEROFF);
				if((SystemStatus & SS_POWEROFF) != 0) {
					//Really need to power off!
//					if((SystemStatus & SS_NEWDATA) == 0) {
//						/*Terminate on error, is shown by SS_NEWDATA flag. When it is 0 then
//						we terminate because there was a communication error.*/
//						/*Need to calculate the time of delay since next communication retry.
//						The concept is simple. If the time of normal retry is smaller than
//						that set by default definitions, we are going to use the update time.
//						If the defined retry time is smaller we are going to use the defined
//						retry time.*/
//						NewTime = ((SystemStatus & SS_GSMTYPEREQ) == 0 ?
//							DEFSETRETRY : DEFSIGRETRY);
//						/*If the defined retry time is bigger than UpdateTime (in seconds)
//						SMSTimer is negative.*/
//						if(NewTime > (UpdateTime *60)) {
//							NewTime = UpdateTime *60;//New time interval in seconds
//						}
//					} else {
//						//Normal Power Off
//						NewTime = UpdateTime *60;	//New time interval in seconds
//						//The clock will trigger again the FSM for Signal HTTP request
//						ClkEvent = (EV_GPRSFLG | EV_HTTPSIG);
//					}
//					Clock_setTimeout(ReGSMClk, NewTime *1000000 /Clock_tickPeriod);
//					Clock_start(ReGSMClk);
					//Clear flags set for this state
					SystemStatus &= ~SS_POWEROFF;
					/*GPRS communications ends and have to power off the module, so light off
					the green led and push the power key of the GSM module.*/
					PIN_setOutputValue(ledPinHandle, GPRS_ACTIVELED, 0);
					PIN_setOutputValue(GSMPinHandle, GSM_PWRKEY, 1);
					//Reset the status flags for GSM UART and prepare to enter the next state
					GSMSerialStatus &= ~(GSMS_STATUS | GSMS_CTS);
					GsmFsmState = GFSM_POFFPUSH;
					//The power key must stay pushed for 1 second to power off the module
//					Task_sleep(1000000/Clock_tickPeriod);
					Semaphore_pend(GPRSSem, 1000000 / Clock_tickPeriod);
				}
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_POFFPUSH:				//Power button is pressed. We need to release it
				DEBUGENTER(GFSM_POFFPUSH);
				//Release the power key and light off the red led (it will be lit in case of
				// error
				PIN_setOutputValue(GSMPinHandle, GSM_PWRKEY, 0);
				PIN_setOutputValue(ledPinHandle, GSM_ERRORLED, 0);
				GsmFsmState = GFSM_POFFREL;	//Set the new state of the FSM
				//Wait another 1 Sec with power button released for the module to settle down
//				Task_sleep(1000000/Clock_tickPeriod);
				Semaphore_pend(GPRSSem, 1000000 / Clock_tickPeriod);
				DEBUGEXIT(0);

			//--------------------------------------
			case GFSM_POFFREL:				//Power off is released, so
				DEBUGENTER(GFSM_POFFREL);
//				SystemStatus &= ~SS_GPRSDELAY;	//Clear the timeout flag
//				SystemStatus |= SS_COMMENDED;	//Flag that communication module ended
//				DisableGSMSerial();
				UART_close(GSMUartHandle);	//Disable the UART module to conserve power
				//Light off the GSM Usage led (Blue) and cut the power to the module
				PIN_setInterrupt(GSMPinHandle, GSM_STATUS | PIN_IRQ_DIS);
				PIN_setOutputValue(ledPinHandle, GSM_ACTIVELED, 0);
				PIN_setOutputValue(GSMPinHandle, GSM_VCC, 0);
				Event_post(MainEvHandle,
					((SystemStatus & SS_NEWDATA) == 0)?GPRSEV_ERROR:GPRSEV_COMMOK);
				GsmFsmState = GFSM_OFF;		//Flag that now the module is totally off.
				//Flag that GSM module is not in use anymore and Serial status is off
				SystemStatus &= ~(SS_GSMON | SS_NEWDATA);
//				GSMSerialStatus &= ~(GSMS_CTS | GSMS_STATUS);//Status and CTS are off now
				DEBUGEXIT(0);

			//--------------------------------------
			default: break;					//All other states are invalid

			}
        } while (SystemStatus & SS_NOEXIT);	//Stay in the loop only if SS_NOEXIT is set
	}
}
