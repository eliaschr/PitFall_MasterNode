/***** Includes *****/
#include <MainTask.h>
#include <RF/RadioTask.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>

/* Drivers */
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"
#include "PitFall_MasterNode.h"

#include "GenericFuncs.h"
#include "RTC/RTCFuncs.h"
#include "RF/RadioProtocol.h"
#include "GSM/UART_GPRS.h"
#include "Flash/AppFlash.h"


/***** Defines *****/
#define MAIN_TASK_STACK_SIZE	1024
#define MAIN_TASK_PRIORITY		2


/***** Variable declarations *****/
//static Task_Params concentratorTaskParams;
Task_Struct MainTaskStruct;					//Structure of Main Task
static uint8_t MainTaskStack[MAIN_TASK_STACK_SIZE];

Event_Struct MainEvStruct;					//Event queue structure for main task
Event_Handle MainEvHandle;					//Handle of main tasks's event queue

/*The RTC of the system cannot be used by this code because it is in use by TI-RTOS. So, the
usage of one shot clock objects is necessary.*/
Clock_Handle ReGSMClk;
Clock_Handle ReRFClk;

union RFPacket* CurrPacket;					//Packet to be processed from the Packet Queue

#pragma DATA_ALIGN(EvArray, 4)
NodeEvent EvArray[MAXNODEEVENTS];			//Array of events from nodes
NodePars NodesParsArr[MAXNODES];			//Array of nodes' parameters
uint16_t NodesEvCnt;						//Counter of events stored in the events array
uint16_t VBattLvl;							//Current battery measurement

/***** Prototypes *****/
static void MainTaskFxn(UArg arg0, UArg arg1);
void GSMStartSMS(void);						//Callback function for Select key button. It
											// starts GSM FSM to receive SMS messages
void GSMStartSettings(void);				//Callback function for Up key button. It starts
											// GPRS FSM to send a "Settings" request to server


/***** Function definitions *****/
//********************************************************************************************
/*Callback function for SELECT key button. It triggers the GSM FSM to start SMS reception and
parsing. SMS Parsing does not stop the GPRS repetition clock, so when it is time to repeat a
GPRS communication, it will happen at the predefined time.*/
void GSMStartSMS(void) {
	if((SystemStatus & SS_GSMON) == 0) {
		Event_post(GPRSEvt, EV_SMSRECEIVE);
	}
}


//********************************************************************************************
/*Callback function for UP key button. It triggers the GSM FSM to start a "Settings" HTTP
request to get variables from internet host server. There is the possibility of altering the
update time after receiving a Settings HTTP response. This means that the GPRS repetition
clock should be reprogrammed. For that reason it is stopped.*/
void GSMStartSettings(void) {
	if((SystemStatus & SS_GSMON) == 0) {
		Event_post(GPRSEvt, EV_GPRSFLG | EV_HTTPSETS);
		Clock_stop(ReGSMClk);
	}
}


/*********************************************************************************************
Adds an event into the table of events to be sent to the Internet server. The event is added
only of there is space in the events array. It is added in the first empty cell of the array.
An array cell is considered empty if the NodeShortAddr field is DEF_BROADCAST_ADDRESS. The
function takes as input a pointer to a NodeEvent structure that contains all the necessary
data to be stored in the events array.
 */
int16_t AddEvent(NodeEvent* InEv) {
	uint16_t elm;							//First empty element in events array
	uint16_t CurrElm;						//Current element during iteration in database
	uint16_t MaxElm;						//Number of non empty elements scanned

	//Do we have space in the events array?
	if(NodesEvCnt >= MAXNODEEVENTS) {
		return -1;							//No => Just exit with negative value (error)
	}
	/*Need to test if this event is already in our database, by checking its Node Short
	Address, its timestamp and its Event Type. If all three match then we do not add it in our
	database. In the same loop we also try to find the first empty cell in the database. To
	help the loop consume less CPU power and time, MaxElm counts the elements scanned. Every
	time a non-empty cell is found it counts it in, together with the time the first empty
	cell is found. This makes it count NodesEvCnt +1 (that's why the '=' is included together
	with the '<' in its condition in the 'while' command: ...(MaxElm <= NodesEvCnt)).*/
	elm = MAXNODEEVENTS;					//First empty cell seems outside array
	CurrElm = 0;							//Start iterating from the beginning
	MaxElm = 0;								//No non-empty elements scanned
	while((elm <= MAXNODEEVENTS) && (MaxElm <= NodesEvCnt)) {
		if(EvArray[CurrElm].NodeShortAddr == InEv->NodeShortAddr) {
			MaxElm++;
			if((EvArray[CurrElm].timestamp == InEv->timestamp) &&
				(EvArray[CurrElm].eventType == InEv->eventType)) {
				elm = MAXNODEEVENTS +1;
			}
		} else if(EvArray[CurrElm].NodeShortAddr == DEF_BROADCAST_ADDRESS) {
			if(elm == MAXNODEEVENTS) {
				elm = CurrElm;
				MaxElm++;
			}
		} else {
			MaxElm++;
		}
		CurrElm++;
	}
	/*If the event is already in the database the 'elm' variable will have a value greater
	than the number of cells in the events database. If the event is not already in the
	database, 'elm' variable will have the value of the first empty cell, if there is any
	before the end of the list (gap). In case there was no gap in the list and the event is
	not found, elm will have the value of MAXNODEEVENTS.*/
	if(elm > MAXNODEEVENTS) {
		return NodesEvCnt;
	}// else if(elm == MAXNODEEVENTS) {
//		elm = NodesEvCnt;
//	}

	/*Now 'elm' variable contains the index of the first empty cell in the events array, so
	store the new event there.*/
	memcpy(&EvArray[elm], InEv, NODEEVENT_SIZE);
	NodesEvCnt++;							//One more event in this array
	return NodesEvCnt;						//Return the number of events in the events array
}


void MainTask_init(void) {
	Clock_Params clkParams;
	union {
		Task_Params taskParams;
		Event_Params evParams;
	} Pars;

	//Create event used internally for state changes and tasks synchronization
	Event_Params_init(&Pars.evParams);
	Event_construct(&MainEvStruct, &Pars.evParams);
	MainEvHandle = Event_handle(&MainEvStruct);

	//Clock creation for repeating GSM communication
	Clock_Params_init(&clkParams);
	clkParams.startFlag = false;
	ClkEvent = 0;
	ReGSMClk = Clock_create(GSMRestart, DEFUPDTIME *1000000/Clock_tickPeriod, &clkParams,
    	NULL);
	ReRFClk = Clock_create(RFRestart, (*NodTm) * 60 *1000000 / Clock_tickPeriod, &clkParams,
		NULL);

	//Create the main task that synchronizes the other ones
	Task_Params_init(&Pars.taskParams);
	Pars.taskParams.stackSize = MAIN_TASK_STACK_SIZE;
	Pars.taskParams.priority = MAIN_TASK_PRIORITY;
	Pars.taskParams.stack = &MainTaskStack;
	Pars.taskParams.arg0 = 1000000;
	Task_construct(&MainTaskStruct, MainTaskFxn, &Pars.taskParams, NULL);

	//Clear the Node Evens Array and associated variables
	for(NodesEvCnt = 0; NodesEvCnt < MAXNODEEVENTS; NodesEvCnt++) {
		EvArray[NodesEvCnt].timestamp = 0;
		EvArray[NodesEvCnt].temperature = 0;
		EvArray[NodesEvCnt].eventType = 0;
		EvArray[NodesEvCnt].NodeShortAddr = DEF_BROADCAST_ADDRESS;
	}

	//Same with the array of last known node parameters
	NodesParsArr[0].MasterCounter = 0;
	NodesParsArr[0].Battery = 0;
	for(NodesEvCnt = 0; NodesEvCnt < MAXNODES; NodesEvCnt++) {
		NodesParsArr[NodesEvCnt].MasterCounter = 0;
		NodesParsArr[NodesEvCnt].Battery = 0;
		NodesParsArr[NodesEvCnt].SndTm = 0;
		NodesParsArr[NodesEvCnt].StampsNo = 0;
		NodesParsArr[NodesEvCnt].rssi = 0;
		NodesParsArr[NodesEvCnt].flags = 0;
		NodesParsArr[NodesEvCnt].CALADC12_15V_30C = 0;
		NodesParsArr[NodesEvCnt].CALADC12_15V_85C = 0;
	}

	//As NodesEvCnt variable was used as a helper one, now it needs to be initialized also
	NodesEvCnt = 0;
}

static void MainTaskFxn(UArg arg0, UArg arg1) {
	uint32_t ev;							//The event that triggered the event loop
	uint32_t NewGSMTime;					//Helper for the timeout triggering of GPRS Task
	uint32_t StartTime;						//Starting time for Ack reception
	uint32_t CurrInterval;					//Interval from starting timeout to current time
	uint32_t EndInterval;					//Interval till Ack reception expiration
	UInt key;								//Key for interrupt disabling and restoring
	uint16_t CRCCheck;						//CRC Checksum calculation
	uint8_t i, j;							//Helper variables for loops

	RTCInit();								//Initialize the RTC

	/*SELECT button should trigger SMS reception and parsing and UP button should trigger a
	Settings HTTP request. The bounding of the switches must be done AFTER BIOS is started, in
	order to prevent an interrupt signaling before tasks are ready.*/
	registerButtonCb(SMS_BUTTON, GSMStartSMS);
	registerButtonCb(GSM_SETTINGS, GSMStartSettings);

	/*Start GSM activity for SMS reading and GPRS HTTP Settings request. The parameters must
	be set by the server in order for other systems to function synchronized.*/
	//Fire a GSM activity.
	Event_post(GPRSEvt, EV_GPRSFLG | EV_SMSRECEIVE | EV_HTTPSETS);
	ev = 0;
	do {									//Must repeat GSM activity until everything is OK
		//Wait until it is done
		ev = Event_pend(MainEvHandle, Event_Id_NONE, GPRSEV_ALL, BIOS_WAIT_FOREVER);
		switch(ev) {
		case GPRSEV_ERROR:
			/*GPRS communication was terminated because there was a communication error.*/
			/*Need to calculate the time of delay since next communication retry. The concept
			is simple. If the time of normal retry is smaller than that set by default
			definitions, we are going to use the update time. If the defined retry time is
			smaller, we are going to use the defined retry time.*/
			NewGSMTime = DEFSETRETRY;
			/*If the defined retry time is bigger than UpdateTime (in seconds), SMSTimer is
			negative.*/
			if(NewGSMTime > (UpdateTime *60)) {
				NewGSMTime = UpdateTime *60;	//New time interval in seconds
			}
			break;

		case GPRSEV_COMMOK:
			//Normal Power Off
			NewGSMTime = UpdateTime *60;		//New time interval in seconds
			//The clock will trigger again the FSM for Signal HTTP request
			ClkEvent = (EV_GPRSFLG | EV_HTTPSIG);
			break;
		}
		//Need to restart the clock for GPRS communication
		Clock_setTimeout(ReGSMClk, NewGSMTime *1000000 /Clock_tickPeriod);
		Clock_start(ReGSMClk);
	} while(ev != GPRSEV_COMMOK);

	//Start the RF communication. Now RTC settings are set from the server
	Event_post(RFEventHandle, RF_EVENT_START);

	//Going to enter the main loop that synchronizes all the events and does parallel job
	while (1) {
		ev = Event_pend(MainEvHandle, NULL, GPRSEV_ALL | RFEV_ALL, BIOS_WAIT_FOREVER);
		/*Lets check the events that come from GSM Task. Two events are expected:
		GPRSEV_COMMOK is fired from GSM Task when the scheduled communication was successful,
		GPRSEV_ERROR is fired if there was an error. These two are mutually exclusive and
		define the timeout of the GSM repetition for the next communication.*/
		NewGSMTime = 0;
		if((ev & GPRSEV_COMMOK) != 0) {		//GPRS Communication terminated normally
			//Normal Power Off
			NewGSMTime = UpdateTime *60;	//New time interval in seconds
			//The clock will trigger again the FSM for Signal HTTP request
			ClkEvent = (EV_GPRSFLG | EV_HTTPSIG);
		}
		if((ev & GPRSEV_ERROR) != 0) {		//GPRS Communication terminated on error
			NewGSMTime = ((ClkEvent & EV_HTTPSETS) == 0 ?
				DEFSIGRETRY : DEFSETRETRY);
			/*If the defined retry time is bigger than UpdateTime (in seconds), SMSTimer is
			negative.*/
			if(NewGSMTime > (UpdateTime *60)) {
				NewGSMTime = UpdateTime *60;//New time interval in seconds
			}
		}

		/*Other events come from the RF Task. They can be one of the following:
		RFEV_DONE is fired when all RF activity is done and the RF module is set off.
		RFEV_NEWDATA is fired when there is a new packet in the packet queue. The packets are
		processed in this task in order for the RF task to be focused on RF communication. If
		Main Task is the one that processes the incoming RF packets, the RF receiver is
		re-enabled as soon as possible, making the reception of new packets smoother.*/
		while(((ev & RFEV_NEWDATA) != 0) && (PacketQLen > 0)) {
			//Lets get the current packet
			CurrPacket = &PacketQ[PacketQFirst];
			/*Need to calculate the CRC of the packet. This is a step independent of the
			packet type. Most of the packets contain a CRC16 which lies just after the
			header. The header is included in the CRC16 checksum.*/
			CRCCheck = DEF_CRC16;		//Initialize CRC Checksum
			for(i = 0; i < CurrPacket->init.preamp.len; i++) {
				if(i == RFHEAD_SIZE) {	//If i points to CRC value => ...
					i += RFCRC_SIZE;	// ... then skip it. It is not included in final
										// CRC Checksum
				}
				CRCCheck = CalcCRC16(CurrPacket->raw[i +RFPREAMP_SIZE], CRCCheck);
			}
			/*Another point is the source address of the node. It cannot be the address of
			the master node. In that case the address will be used is the broadcast one.*/
			if(CurrPacket->init.header.sourceAddress == DEF_MASTER_ADDRESS) {
				CurrPacket->init.header.sourceAddress = DEF_BROADCAST_ADDRESS;
			}
			/*Now the packet must be parsed according to its type. Lets begin with the
			registration request packet type.*/
			if((CurrPacket->init.header.packetType == RFPacket_Register) ||
				(CurrPacket->init.header.packetType == RFPacket_Data) ||
				(CurrPacket->init.header.packetType == RFPacket_Ack) ||
				(CurrPacket->init.header.packetType == RFPacket_GetInfo)) {
				if(CurrPacket->init.header.packetType == RFPacket_Register) {
					ParseRegPacket(CurrPacket, CRCCheck);
				} else if(CurrPacket->init.header.packetType == RFPacket_Data) {
					ParseDataPacket(CurrPacket, CRCCheck);
				} else if(CurrPacket->init.header.packetType == RFPacket_Ack) {
					j = AckQStrt;
					for(i = 0; i < AckQLen; i++) {
						if((AckQueue[j].NodeAddr == CurrPacket->ack.header.sourceAddress) &&
							(AckQueue[j].packetSession == CurrPacket->ack.packetSession) &&
							(AckQueue[j].packetNo == CurrPacket->ack.packetNo)) {
							break;
						}
						j++;
						if(j == ACKQ_SIZE) {
							j = 0;
						}
					}
					if(i < AckQLen) {
						NodesParsArr[CurrPacket->ack.header.sourceAddress].flags = 0;
						AckQueue[j].NodeAddr = 0;
						do {
							if(AckQueue[AckQStrt].NodeAddr == 0) {
								key = Hwi_disable();
								AckQStrt++;
								AckQClk++;
								AckQLen--;
								Hwi_restore(key);
							} else {
								StartTime = AckQueue[AckQClk].timeTicks;
								CurrInterval = Clock_getTicks() -StartTime;
								EndInterval = StartTime +RFACK_TIMEOUT -StartTime;
								if(CurrInterval >= EndInterval) {
									key = Hwi_disable();
									AckQStrt++;
									AckQClk++;
									AckQLen--;
									Hwi_restore(key);
								} else {
									Clock_stop(RFTimeoutClk);
									Clock_setTimeout(RFTimeoutClk, EndInterval -CurrInterval);
									Clock_start(RFTimeoutClk);
									break;
								}
							}
						} while(AckQLen > 0);
						if(AckQLen == 0) {
							RFStatusFlags &= ~RFS_EXPECTACK;
						}
					}
				} else if(CurrPacket->init.header.packetType == RFPacket_GetInfo) {
					ParseInfoPacket(CurrPacket, CRCCheck);
				}
				//Remove this packet from the Packet Queue
				key = Hwi_disable();		//This is a critical section. Start and Length
											// variables should be altered together
				PacketQFirst++;				//Advance the starting pointer and
				PacketQLen--;				//... decrease the queue length (one packet less)
				if(PacketQFirst >= PACKETQ_SIZE) {
					//Starting pointer passed the end of queue? =>
					PacketQFirst = 0;		//Just revert to its beginning
				}
				Hwi_restore(key);			//Critical section ends here
			}
		}

		/*When the RF task ends, we need to schedule its restart. In order to avoid concurrent
		activation of both RF and GPRS tasks, the RF Clock is scheduled to expire 5 minutes
		before its real interval. Then it raises the flag that delays the GPRS task and is
		rescheduled for the rest 5 minutes, where there is a firing of the RF task for RF data
		exchange. If there is an expiration of the GPRS clock, during the period the GPRS
		delay flag is raised, the clock callback function does not fire the GPRS task, but it
		also sets the flag that shows the need for GPRS communication firing just after RF
		task ends. When the RF task ends, it sends an RFEV_DONE event. The system checks for
		the need of the GPRS communication and if set, it fires the callback function of the
		GPRS clock to start the GPRS communication.*/
		if((ev & RFEV_DONE) != 0) {
			StartTime = RTCGetCurrTime();
			CurrInterval = (NextWakeUpTime -StartTime -INHIBITGSM_TIMEOUT) * 1000000 /
				Clock_tickPeriod;
			Clock_setTimeout(ReRFClk, CurrInterval);
			Clock_start(ReRFClk);
			if((RFStatusFlags & RFS_DELAYGPRS) != 0) {
				RFStatusFlags &= ~RFS_DELAYGPRS;
				if((RFStatusFlags & RFS_NEEDGPRS) != 0) {
					RFStatusFlags &= ~RFS_NEEDGPRS;
					GSMRestart(0);
				}
			}
		}

		//Need to restart the clock for GPRS communication if needed
		if(NewGSMTime != 0) {
			Clock_stop(ReGSMClk);
			Clock_setTimeout(ReGSMClk, NewGSMTime *1000000 /Clock_tickPeriod);
			Clock_start(ReGSMClk);
		}
	}
}
