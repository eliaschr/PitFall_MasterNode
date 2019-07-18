/***** Includes *****/
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include "RF/RadioTask.h"

#include <ti/sysbios/BIOS.h>

#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
//#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"
#include "PitFall_MasterNode.h"
#include "MainTask.h"

#include "RF/EasyLink/EasyLink.h"
#include "RF/RadioProtocol.h"
#include "RTC/RTCFuncs.h"
#include "Flash/AppFlash.h"
#include "GenericFuncs.h"


/***** Defines *****/
#define RF_TASK_STACK_SIZE					1024
#define RF_TASK_PRIORITY					3

#define RF_EVENT_ALL						0xFFFFFFFF
//#define RF_EVENT_START					Event_Id_00
#define RF_EVENT_BEACON						Event_Id_01
#define RF_EVENT_VALID_PACKET_RECEIVED		Event_Id_02
#define RF_EVENT_INVALID_PACKET_RECEIVED	Event_Id_03
#define RF_EVENT_NODESCAN					Event_Id_04
#define RF_EVENT_SLEEP						Event_Id_31

//#define RF_MAX_RETRIES						2
//#define NORF_ACK_TIMEOUT					(160)


/***** Type declarations *****/


/***** Variable declarations *****/
static Task_Params RFTaskParams;
Task_Struct RFTask; /* not static so you can see in ROV */
static uint8_t RFTaskStack[RF_TASK_STACK_SIZE];

Event_Struct RFEvent;  /* not static so you can see in ROV */
Event_Handle RFEventHandle;

Clock_Handle RFBeaconClk;
//Clock_Params RFBeaconPars;
Clock_Handle RFScanClk;
//Clock_Params RFScanPars;
Clock_Handle RFFlashClk;
//Clock_Params RFFlashPars;
Clock_Handle RFTimeoutClk;
//Clock_Params RFTimeoutPars;
Clock_Handle RFLongPressClk;
//Clock_Params RFLongPressPars;

volatile uint32_t RFStatusFlags;
uint32_t EndBeaconTime;
uint32_t NextWakeUpTime;
//static RFPacketCb packetReceivedCallback;
AckBack AckQueue[ACKQ_SIZE];
static union RFPacket* latestRxPacket;
static union RFPacket AnswerPacket;
union RFPacket PacketQ[PACKETQ_SIZE];
static EasyLink_TxPacket txPacket;
struct BeaconPacket beaconPacket;
volatile uint8_t PacketQFirst;
volatile uint8_t PacketQLen;
static uint8_t MasterAddress;
volatile uint8_t AckQStrt;
volatile uint8_t AckQLen;
volatile uint8_t AckQClk;
//static int8_t latestRssi;
//static uint8_t latestLen;


/***** Prototypes *****/
void RFRegisterNode(void);
void StopScanMode(UArg InArg);
void FlashScanLed(UArg InArg);
void AckTimeout(UArg InArg);
static void SendBeaconClk(UArg InArg);
static void LongPress(UArg InArg);
static void RFTaskFxn(UArg arg0, UArg arg1);
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);
//static void notifyPacketReceived(union RFPacket* latestRxPacket);
static void sendAck(uint8_t latestSourceAddress);
static void sendRegAck(uint8_t latestSourceAddress);
static void sendBeacon(void);
static void sendNodeInfo(uint8_t latestSourceAddress);


/***** Function definitions *****/
void RFRegisterNode(void) {
	if(PIN_getInputValue(RF_BUTTON) == 0) {
		if((RFStatusFlags & RFS_SYNC) == 0) {
			RFStatusFlags |= RFS_SYNC;
			Clock_start(RFScanClk);
//			Clock_start(RFFlashClk);
//			PIN_setOutputValue(ledPinHandle, RF_SYNCLED, 1);
		}
		if((RFStatusFlags & RFS_ONAIR) == 0) {
			Event_post(RFEventHandle, RF_EVENT_NODESCAN);
		}
		if((RFStatusFlags & RFS_REGISTER) == 0) {
			Clock_start(RFLongPressClk);
		}
	} else {
		Clock_stop(RFLongPressClk);
	}
}


void StopScanMode(UArg InArg) {
	if((RFStatusFlags & (RFS_ONAIR | RFS_SYNC)) == (RFS_ONAIR | RFS_SYNC)) {
		Clock_stop(RFScanClk);
		Clock_stop(RFFlashClk);
		RFStatusFlags &= ~(RFS_SYNC | RFS_REGISTER);
		PIN_setOutputValue(ledPinHandle, RF_SYNCLED, 0);
	}
}


void RFRestart(UArg InArg) {
	if((RFStatusFlags & RFS_DELAYGPRS) == 0) {
		RFStatusFlags |= RFS_DELAYGPRS;
		Clock_stop(ReRFClk);
		Clock_setTimeout(ReRFClk, INHIBITGSM_TIMEOUT * 1000000 / Clock_tickPeriod);
		Clock_start(ReRFClk);
	} else {
		Event_post(RFEventHandle, RF_EVENT_START);
	}
}


void FlashScanLed(UArg InArg) {
	if((RFStatusFlags & (RFS_ONAIR | RFS_SYNC)) == (RFS_ONAIR | RFS_SYNC)) {
		PIN_setOutputValue(ledPinHandle, RF_SYNCLED,
			!PIN_getOutputValue(RF_SYNCLED));
		Clock_start(RFFlashClk);
	} else {
		PIN_setOutputValue(ledPinHandle, RF_SYNCLED, 0);
	}
}


void AckTimeout(UArg InArg) {
	uint32_t StartTime;
	uint32_t CurrInterval;
	uint32_t EndInterval;
	UInt key;

	Clock_stop(RFTimeoutClk);
	AckQClk++;
	if(AckQClk >= ACKQ_SIZE) {
		AckQClk -= ACKQ_SIZE;
	}
	if(AckQClk >= AckQStrt) {
		key = Hwi_disable();
		AckQLen -= AckQClk -AckQStrt;
		AckQStrt = AckQClk;
		Hwi_restore(key);
	} else {
		key = Hwi_disable();
		AckQLen -= AckQClk +ACKQ_SIZE -AckQStrt;
		AckQStrt = AckQClk;
		Hwi_restore(key);
	}
	while(AckQLen > 0) {
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
				Clock_setTimeout(RFTimeoutClk, EndInterval -CurrInterval);
				Clock_start(RFTimeoutClk);
				break;
			}
		}
	}
	if(AckQLen == 0) {
		RFStatusFlags &= ~RFS_EXPECTACK;
	}
}


static void SendBeaconClk(UArg InArg) {
	beaconPacket.CurrTimeStamp = RTCGetCurrTime();
	if(((RFStatusFlags & RFS_SYNC) == 0) &&
		(beaconPacket.CurrTimeStamp >= EndBeaconTime)) {
		Event_post(RFEventHandle, RF_EVENT_SLEEP);
	} else {
		Event_post(RFEventHandle, RF_EVENT_BEACON);
		Clock_start(RFBeaconClk);
	}
}


static void LongPress(UArg InArg) {
	if((RFStatusFlags & RFS_REGISTER) == 0) {
		RFStatusFlags |= RFS_REGISTER;
		Clock_start(RFFlashClk);
		PIN_setOutputValue(ledPinHandle, RF_SYNCLED, 1);
		Clock_stop(RFScanClk);
		Clock_start(RFScanClk);
	}
}


void InitRFTask(void) {
	Clock_Params ClkPars;

	/* Create event used internally for state changes */
	Event_Params eventParam;
	Event_Params_init(&eventParam);
	Event_construct(&RFEvent, &eventParam);
	RFEventHandle = Event_handle(&RFEvent);

	/* Create the radio protocol task */
	Task_Params_init(&RFTaskParams);
	RFTaskParams.stackSize = RF_TASK_STACK_SIZE;
	RFTaskParams.priority = RF_TASK_PRIORITY;
	RFTaskParams.stack = &RFTaskStack;
	Task_construct(&RFTask, RFTaskFxn, &RFTaskParams, NULL);

//	Clock_Params_init(&RFBeaconPars);
//	RFBeaconPars.startFlag = false;
	Clock_Params_init(&ClkPars);
	ClkPars.startFlag = false;
	RFBeaconClk = Clock_create(SendBeaconClk, RFBEACON_TIMEOUT, &ClkPars, NULL);

//	Clock_Params_init(&RFScanPars);
//	RFScanPars.startFlag = false;
	RFScanClk = Clock_create(StopScanMode, RFSCAN_TIMEOUT, &ClkPars, NULL);

//	Clock_Params_init(&RFFlashPars);
//	RFFlashPars.startFlag = false;
	RFFlashClk = Clock_create(FlashScanLed, RFFLASH_TIMEOUT, &ClkPars, NULL);

//	Clock_Params_init(&RFTimeoutPars);
//	RFTimeoutPars.startFlag = false;
	RFTimeoutClk = Clock_create(AckTimeout, RFACK_TIMEOUT, &ClkPars, NULL);

//	Clock_Params_init(&RFLongPressPars);
//	RFLongPressPars.startFlag = false;
	RFLongPressClk = Clock_create(LongPress, RFLONGKEY_TIMEOUT, &ClkPars, NULL);

	PacketQFirst = 0;
	PacketQLen = 0;
	latestRxPacket = PacketQ;

	beaconPacket.header.packetType = RFPacket_Beacon;
	beaconPacket.header.sourceAddress = DEF_MASTER_ADDRESS;
	EasyLink_getIeeeAddr(beaconPacket.SourceMAC);

	PIN_setInterrupt(swPinHandle, RF_BUTTON | PIN_IRQ_BOTHEDGES);
	registerButtonCb(RF_BUTTON, RFRegisterNode);
	RFStatusFlags = 0;
	AckQStrt = 0;
	AckQLen = 0;
}


//void RFTask_registerPacketCb(RFPacketCb callback) {
//	packetReceivedCallback = callback;
//}


static void RFTaskFxn(UArg arg0, UArg arg1) {
	EasyLink_Status stat;
	uint32_t ev;
//	NodeEvent NewEvent;
//	uint16_t CRCCheck;
//	uint8_t ShortAddr, i;

	MasterAddress = DEF_MASTER_ADDRESS;

	/*Set up the answer packet. This packet will be used as Ack, NAck, RegisterAck,
	RegisterNAck or NodeInfo. In every case the source address is always the same.*/
	AnswerPacket.init.header.sourceAddress = MasterAddress;

	while (1) {
		/*Here we use EasyLink framework to create our communication protocol. The problem is
		that after some time the RF module is not in use, it falls asleep to preserve power
		and EasyLink cannot wake it up. In that case the process of RF seems to wait forever.
		To override this problem we have to initialize EasyLink framework every time there is
		a new RF cycle in order to avoid lockups!*/
		ev = Event_pend(RFEventHandle, Event_Id_NONE, RF_EVENT_START | RF_EVENT_NODESCAN,
			BIOS_WAIT_FOREVER);

		//Initialize EasyLink for the needed modulation.
		stat = EasyLink_init(DEF_MODULATION);
		if(stat != EasyLink_Status_Success) {
			Halt_abort("EasyLink_init failed", &stat);
		}

		//Set RF frequency to 868MHz.
		stat = EasyLink_setFrequency(DEF_FREQUENCY);
		if(stat != EasyLink_Status_Success) {
			Halt_abort("EasyLink_setFrequency failed", &stat);
		}

		/*Set filter of accepted target addresses from remote nodes. It must be the master
		address. The master node does not listen to any other address (not even the broadcast
		one).*/;
		EasyLink_enableRxAddrFilter(&MasterAddress, 1, 1);

		/*Lets prepare a beacon packet to be sent. This will be the first one and will contain
		the timestamps for beacon end and next time an RF communication will be start again.
		During the whole RF timeslot, beacons will be sent every 10ms.*/
		beaconPacket.CurrTimeStamp = RTCGetCurrTime();

		RFStatusFlags |= RFS_ONAIR;			//Flag that RF now is alive and running
		ev |= RF_EVENT_BEACON;				//Must start by sending a beacon packet

		//Enter asynchronous receive mode
//		stat = EasyLink_receiveAsync(rxDoneCallback, 0);
//		if(stat != EasyLink_Status_Success) {
//			Halt_abort("Leading EasyLink_receiveAsync failed");
//		}

		do {
			if((ev & RF_EVENT_START) != 0) {
				EndBeaconTime = beaconPacket.CurrTimeStamp + RFBEACON_DURATION;
				NextWakeUpTime = beaconPacket.CurrTimeStamp + (*NodTm) * 60;
				beaconPacket.NextWakeUpTime = NextWakeUpTime;
			}

			if((ev & RF_EVENT_BEACON) != 0) {
				EasyLink_abort();			//Stop reception in order to start a transmission
				beaconPacket.CurrTimeStamp = RTCGetCurrTime();
				sendBeacon();
				Clock_start(RFBeaconClk);
				PIN_setOutputValue(ledPinHandle, RF_ACTIVITYLED,
					!PIN_getOutputValue(RF_ACTIVITYLED));
			}

			if((ev & RF_EVENT_NODESCAN) != 0) {
				RFStatusFlags |= RFS_ONAIR;
			}

			stat = EasyLink_receiveAsync(rxDoneCallback, 0);
			if(stat == EasyLink_Status_Busy_Error) {
				EasyLink_abort();
				stat = EasyLink_receiveAsync(rxDoneCallback, 0);
			}
			if(stat != EasyLink_Status_Success) {
				Halt_abort("EasyLink_receiveAsync re-entering failed", &stat);
			}

			ev = Event_pend(RFEventHandle, Event_Id_NONE, RF_EVENT_ALL, BIOS_WAIT_FOREVER);
		} while((ev & RF_EVENT_SLEEP) == 0);
		RFStatusFlags &= ~RFS_ONAIR;
		PIN_setOutputValue(ledPinHandle, RF_ACTIVITYLED, 0);
		EasyLink_abort();
		Event_post(MainEvHandle, RFEV_DONE);
	}
}


static void sendBeacon(void) {
	EasyLink_Status stat;

	/*Set Destination Address to 0, as it is a broadcast packet. Use EasyLink layers
	destination address capability.*/
	txPacket.dstAddr[0] = 0;

	/*Copy preset beacon packet to payload, skipping the destination address byte. Note that
	the EasyLink API will implicitly both add the length byte and the destination address
	byte.*/
	memcpy(txPacket.payload, &beaconPacket.header, RFBEACON_SIZE);
	txPacket.len = RFBEACON_SIZE;

	/*Send packet.*/
//	EasyLink_abort();
	stat = EasyLink_transmit(&txPacket);
	if(stat != EasyLink_Status_Success) {
		Halt_abort("Beacon EasyLink_transmit failed", &stat);
	}
//	stat = EasyLink_receiveAsync(rxDoneCallback, 0);
//	if(stat != EasyLink_Status_Success) {
//		Halt_abort("EasyLink_receiveAsync after beacon failed");
//	}
}


static void sendAck(uint8_t latestSourceAddress) {
	AckBack* AckQItem;						//Points to the current empty cell in Ack queue
	EasyLink_Status stat;					//Return status of EasyLink commands
	uint16_t CRCCheck;						//Helper to calculate the CRC16 of the packet
	uint8_t i;								//Helper counter for "for" loops
	uint8_t tmpLen;							//Helper variable to keep the length of the packet

	//Set the destination address of the packet.
	txPacket.dstAddr[0] = latestSourceAddress;

	tmpLen = RFACK_SIZE;					//Assume the answering packet is an Ack one
	if(AnswerPacket.init.header.packetType == RFPacket_PAck) {
		tmpLen = RFPACK_SIZE;				//Change the size if it is a PAck packet
	} else if(AnswerPacket.init.header.packetType == RFPacket_NAck) {
		tmpLen = RFNACK_SIZE;				//Change the size if it is a NAck packet
	}
	//Ack packet has a CRC field, so lets calculate and add it in the packet
	CRCCheck = DEF_CRC16;					//Initialize CRC Checksum
	for(i = 0; i < tmpLen; i++) {
		if(i == RFHEAD_SIZE) {				//If i points to CRC value => ...
			i += RFCRC_SIZE;				// ... then skip it. It is not included in final
											// CRC Checksum
		}
		CRCCheck = CalcCRC16(AnswerPacket.raw[i +RFPREAMP_SIZE], CRCCheck);
	}
	/*Ack and PAck packets, both have the CRC value at the same place. So the following is
	correct, as AnswerPacket.ack.CRC and AnswerPacket.pack.CRC point to the same byte in the
	packet.*/
	AnswerPacket.ack.CRC = CRCCheck;

	/*Copy ACK packet to payload, skipping the destination address byte. Note that the
    EasyLink API will implicitly both add the length byte and the destination address.*/
	memcpy(txPacket.payload, &AnswerPacket.raw[RFPREAMP_SIZE], tmpLen);
	txPacket.len = tmpLen;

	/*Send packet.*/
	EasyLink_abort();
	stat = EasyLink_transmit(&txPacket);
	if (stat != EasyLink_Status_Success) {
		if(AnswerPacket.init.header.packetType == RFPacket_Ack) {
			Halt_abort("Ack EasyLink_transmit failed", &stat);
		} else if(AnswerPacket.init.header.packetType == RFPacket_PAck){
			Halt_abort("PAck EasyLink_transmit failed", &stat);
		} else {
			Halt_abort("NAck EasyLink_transmit failed", &stat);
		}
	}
	//If the packet sent has any flags set, then we should expect an Ack back from the node
	if((AnswerPacket.init.header.packetType != RFPacket_NAck) &&
		((AnswerPacket.pack.flags & PF_RESET) != 0)) {
		//Prepare the AckQ pointer for the newly expected Ack Packet from a node
		i = AckQStrt + AckQLen;
		if(i > ACKQ_SIZE) {
			i -= ACKQ_SIZE;
		}
		AckQItem = &AckQueue[i];
		//Prepare the AckQueue Item data
		AckQItem->NodeAddr = latestSourceAddress;
		AckQItem->flags = AnswerPacket.pack.flags;
		AckQItem->packetNo = AnswerPacket.pack.packetNo;
		AckQItem->packetSession = AnswerPacket.pack.packetSession;
		AckQItem->timeTicks = Clock_getTicks();
		//If the clock is not running then start it
		if((RFStatusFlags & RFS_EXPECTACK) == 0) {
			AckQClk = i;
			RFStatusFlags |= RFS_EXPECTACK;
			Clock_start(RFTimeoutClk);
		}
	}

	//Re-enter Async receive mode
	stat = EasyLink_receiveAsync(rxDoneCallback, 0);
	if(stat != EasyLink_Status_Success) {
		Halt_abort("EasyLink_receiveAsync after (N|P)Ack failed", &stat);
	}
}


static void sendRegAck(uint8_t latestSourceAddress) {
	uint16_t CRCCheck;						//Helper to calculate the CRC16 of the packet
	uint8_t i, iMax;						//Helper counter for "for" loops and its maximum
											// value
	EasyLink_Status stat;					//Status of EasyLink commands

	//Set the destination address of the packet.
	txPacket.dstAddr[0] = latestSourceAddress;

	/*RegAck and RegNAck packets have a CRC field, so lets calculate it. The maximum number of
	bytes in the packet depends on its type (RegisterAck or RegisterNAck).*/
	iMax = RFREGACK_SIZE;					//Assume a RegisterAck packet
	//Change the size of the packet only if it is a RegisterNAck one
	if(AnswerPacket.init.header.packetType == RFPacket_RegisterNAck) {
		iMax = RFREGNACK_SIZE;				//Change the size if it is a RegisterNAck packet
	}
	CRCCheck = DEF_CRC16;					//Initialize CRC Checksum
	for(i = 0; i < iMax; i++) {
		if(i == RFHEAD_SIZE) {				//If i points to CRC value => ...
			i += RFCRC_SIZE;				// ... then skip it. It is not included in final
											// CRC Checksum
		}
		CRCCheck = CalcCRC16(AnswerPacket.raw[i +RFPREAMP_SIZE], CRCCheck);
	}
	AnswerPacket.regAck.CRC = CRCCheck;

	/*Copy produced packet to payload, skipping the destination address byte. Note that the
    EasyLink API will implicitly both add the length byte and the destination address.*/
	memcpy(txPacket.payload, &AnswerPacket.raw[RFPREAMP_SIZE], iMax);
	txPacket.len = iMax;

	/*Send packet.*/
	EasyLink_abort();
	stat = EasyLink_transmit(&txPacket);
	if (stat != EasyLink_Status_Success) {
		if(AnswerPacket.init.header.packetType == RFPacket_RegisterNAck) {
			Halt_abort("RegisterNAck EasyLink_transmit failed", &stat);
		}
		Halt_abort("RegisterAck EasyLink_transmit failed", &stat);
	}
	stat = EasyLink_receiveAsync(rxDoneCallback, 0);
	if(stat != EasyLink_Status_Success) {
		Halt_abort("EasyLink_receiveAsync after Register(N)Ack failed", &stat);
	}
}


static void sendNodeInfo(uint8_t latestSourceAddress) {
	AckBack* AckQItem;						//Points to the current empty cell in Ack queue
	uint16_t CRCCheck;						//Helper to calculate the CRC16 of the packet
	uint8_t i;								//Helper counter for "for" loops
	EasyLink_Status stat;					//Status of EasyLink commands

	//Set the destination address of the packet.
	txPacket.dstAddr[0] = latestSourceAddress;

	//Set the RF restarting activity time set to the master
	AnswerPacket.nodeInfo.RadioTm = *NodTm;

	//NodeInfo packet has a CRC field, so lets calculate and add it in the packet
	CRCCheck = DEF_CRC16;					//Initialize CRC Checksum
	for(i = 0; i < RFNODEINFO_SIZE; i++) {
		if(i == RFHEAD_SIZE) {				//If i points to CRC value => ...
			i += RFCRC_SIZE;				// ... then skip it. It is not included in final
											// CRC Checksum
		}
		CRCCheck = CalcCRC16(AnswerPacket.raw[i +RFPREAMP_SIZE], CRCCheck);
	}
	AnswerPacket.nodeInfo.CRC = CRCCheck;

	/*Copy the produced packet to payload, skipping the destination address byte. Note that
    EasyLink API will implicitly both add the length byte and the destination address.*/
	memcpy(txPacket.payload, &AnswerPacket.raw[RFPREAMP_SIZE], RFNODEINFO_SIZE);
	txPacket.len = RFNODEINFO_SIZE;

	/*Send packet.*/
	EasyLink_abort();
	stat = EasyLink_transmit(&txPacket);
	if (stat != EasyLink_Status_Success) {
		Halt_abort("NodeInfo EasyLink_transmit failed", &stat);
	}

	//If the packet sent has any flags set, then we should expect an Ack back from the node
	if((AnswerPacket.nodeInfo.flags & PF_RESET) != 0) {
		//Prepare the AckQ pointer for the newly expected Ack Packet from a node
		i = AckQStrt + AckQLen;
		if(i > ACKQ_SIZE) {
			i -= ACKQ_SIZE;
		}
		AckQItem = &AckQueue[i];
		//Prepare the AckQueue Item data
		AckQItem->NodeAddr = latestSourceAddress;
		AckQItem->flags = AnswerPacket.nodeInfo.flags;
		AckQItem->packetNo = 0;
		AckQItem->packetSession = 0;
		AckQItem->timeTicks = Clock_getTicks();
		//If the clock is not running then start it
		if((RFStatusFlags & RFS_EXPECTACK) == 0) {
			AckQClk = i;
			RFStatusFlags |= RFS_EXPECTACK;
			Clock_start(RFTimeoutClk);
		}
	}

	//Finally re-enter Async receive mode
	stat = EasyLink_receiveAsync(rxDoneCallback, 0);
	if(stat != EasyLink_Status_Success) {
		Halt_abort("EasyLink_receiveAsync after NodeInfo packet sending failed", &stat);
	}
}


static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status) {
	UInt key;
	UInt EventType;
	int i;

	/*When there is an EasyLink_abort() function call, the EasyLink framework also calls the
	rxDoneCallback function with the same status (EasyLink_Status_Aborted). Since the firing
	of an abort command comes only from the main program, the process knows that an abort is
	done and the EasyLink framework does not accept any packets until it enters a Receive mode
	again. So, no event firing should be made!*/
	if(status == EasyLink_Status_Aborted) {
		return;
	}

	EventType = RF_EVENT_INVALID_PACKET_RECEIVED;
	if(PacketQLen >= PACKETQ_SIZE) {		//No space in packet queue?
		//Need to exit by posting an Invalid Packet event and ignore this packet
		Event_post(RFEventHandle, EventType);
		return;
	}
	//If we received a packet successfully we have to parse it
	if (status == EasyLink_Status_Success) {
		//Save the latest RSSI
		latestRxPacket->init.preamp.rssi = (int8_t)rxPacket->rssi;
		latestRxPacket->init.preamp.len = rxPacket->len;

		//Check if this is an acceptable packet
		/*First we have to point to the payload of the incoming EasyLink packet, in order to
		parse the data as the correct communication packet. Remember that the communication
		packet is embedded in an EasyLink one as the payload. Also, the first part of a packet
		in memory contains a small preamp, which must be ignored as the received packet
		contains only the fields from header part to end of packet.*/
		memcpy(&latestRxPacket->raw[RFPREAMP_SIZE], rxPacket->payload, rxPacket->len);

		//Lets check if this is a known and acceptable packet
		/*The valid packets for the master are RFPacket_Register, RFPacket_Data and
		RFPacket_GetInfo.*/
		if(latestRxPacket->init.header.packetType == RFPacket_Register) {
			/*The RFPacket_Register can be received while in SCAN mode, or in normal mode. In
			normal mode, only the already registered nodes will receive a RegisterAck packet;
			the unregistered ones will receive a RegisterNAck one. Another point is that the
			MAC address of the master in the packet must match the one of this master. If this
			is not true then no reply should be sent (not even a NAck).*/
			for(i = 0; (i < 8) &&
				(beaconPacket.SourceMAC[i] == latestRxPacket->regNode.MasterMAC[i]); i++);
			//At this point if i == 8 then the two MAC addresses match
			if(i == 8) {
				EventType = RF_EVENT_VALID_PACKET_RECEIVED;
			}
		} else if(latestRxPacket->init.header.packetType == RFPacket_Data) {
			/*Data packets are always accepted as EasyLink filters the short address of the
			destination node (master node) by itself.*/
			EventType = RF_EVENT_VALID_PACKET_RECEIVED;
		} else if((latestRxPacket->init.header.packetType == RFPacket_Ack) &&
			((RFStatusFlags & RFS_EXPECTACK) != 0)) {
			/*When the master sends data to the Internet server, it accepts parameters for a
			node. The node is notified for these parameters through the "flags" field of an
			Ack packet that the master sends to the node to notify that it accepted all of the
			events send by the node. For the master to be able to understand that the node
			got the parameters, the node sends back an Ack packet to indicate the acceptance
			of the parameters the internet server requested. The Ack packet from the node is
			transmitted only if the "flags" field in the Ack (or PAck) packet the master sent
			is not cleared. If an Ack is not sent back to the master in that case, the master
			will continue to send the not empty flags field to the node.*/
			//Lets see if we expect an Ack from this node
			EventType = RF_EVENT_VALID_PACKET_RECEIVED;
		} else if(latestRxPacket->init.header.packetType == RFPacket_GetInfo) {
			/*When a slave node does not have data to send to the master, it just issues a Get
			Information packet. Using that, it sends the necessary parameters of the node,
			like battery level, etc, and requests the parameters the server sets for that
			node. A GetInfo packet is always answered by a NodeInfo packet.*/
			EventType = RF_EVENT_VALID_PACKET_RECEIVED;
		}
	}

	/*Now lets include the copied packet in packet queue. This must be done only if the packet
	is valid.*/
	if(EventType == RF_EVENT_VALID_PACKET_RECEIVED) {
		key = Hwi_disable();
		PacketQLen++;
		i = PacketQFirst + PacketQLen;
		if(i >= PACKETQ_SIZE) {
			i -= PACKETQ_SIZE;
		}
		latestRxPacket = &PacketQ[i];
		Hwi_restore(key);
		Event_post(MainEvHandle, RFEV_NEWDATA);
	}
	Event_post(RFEventHandle, EventType);
}


void ParseRegPacket(union RFPacket* CurrPacket, uint16_t CRCCheck) {
	int ShortAddr, i;

	//Get prepared for a NAck packet, in case there was an error in CRC
	AnswerPacket.ack.packetNo = 0;
	AnswerPacket.ack.packetSession = 0;
	if(CurrPacket->regNode.CRC != CRCCheck) {
		//CRC Error => Then send a NAck packet.
		AnswerPacket.init.header.packetType = RFPacket_NAck;
		/*Copy the MAC address of the target node. Node may not have a short address yet, so
		the target short address would be the broadcast one and the NodeMAC defines the real
		target node. In case the node is preciously registered the node already has a short
		address. It is the responsibility of the node to send the correct address. In case the
		node claims to be the master, the broadcast address will be used as the target address
		of the NAck packet.*/
		for(i = 0; i < 8; i++) {
			AnswerPacket.ack.NodeMAC[i] = CurrPacket->regNode.NodeMAC[i];
		}
		sendAck(CurrPacket->init.header.sourceAddress);
	} else {
		/*CRC is OK, so we have to find out the correct response. If the master is in SCAN
		mode it has to register the source node and send a RegisterAck packet. If not in SCAN
		mode, then the answer should be a RegisterAck in case the node is already registered,
		or a RegisterNAck in case the node is not in DHCP table.*/
		//Store the rssi of the node in NodeParams
		NodesParsArr[CurrPacket->init.header.sourceAddress -2].rssi =
			CurrPacket->init.preamp.rssi;
		//Prepare a RegisterNAck packet to be ready
		AnswerPacket.init.header.packetType = RFPacket_RegisterNAck;
		//Copy the Node's MAC address
		for(i = 0; i < 8; i++) {
			AnswerPacket.regAck.NodeMAC[i] = CurrPacket->regNode.NodeMAC[i];
		}
		if((RFStatusFlags & RFS_REGISTER) != 0) {
			//In SCAN mode. Try to add the new node in the DHCP table
			i = NodesCnt;					//Temporary value of number of nodes
			ShortAddr = AddNode(CurrPacket->regNode.NodeMAC);
			/*If the new node is added (and not just found) then stop SCAN mode.*/
			if(i != NodesCnt) {
				StopScanMode(0);
			}
		} else {
			//Not in SCAN mode. Just search if this node is registered
			ShortAddr = GetNodeAddress(CurrPacket->regNode.NodeMAC);
		}
		if(ShortAddr != 0) {
			/*Node is in DHCP Table. Convert the answer packet to RegisterAck one.*/
			AnswerPacket.init.header.packetType = RFPacket_RegisterAck;
			AnswerPacket.regAck.NodeShortAddr = ShortAddr;
		}
		sendRegAck(CurrPacket->init.header.sourceAddress);
	}
}


void ParseDataPacket(union RFPacket* CurrPacket, uint16_t CRCCheck) {
	NodeEvent NewEvent;
	int i, AddedStamps;
	int16_t StmpSlot;

	/*At this point the longest reply packet is PAck. Ack and NAck are subsets of a PAck. So,
	lets setup a PAck one to include all of the possible answers. This packet will be flagged
	as NAck first, until CRC check indicates the validity of the packet received.*/
	AnswerPacket.init.header.packetType = RFPacket_NAck;
	AnswerPacket.pack.packetNo = CurrPacket->rfData.packetNo;
	AnswerPacket.pack.packetSession = CurrPacket->rfData.packetSession;
	for(i = 0; i < 8; i++) {
		AnswerPacket.pack.NodeMAC[i] = beaconPacket.SourceMAC[i];
	}
	if(CurrPacket->rfData.CRC == CRCCheck) {
		AnswerPacket.init.header.packetType = RFPacket_PAck;
		AnswerPacket.pack.eventsCount = 0;
		AnswerPacket.pack.flags =
			NodesParsArr[CurrPacket->init.header.sourceAddress -2].flags;

		/*The received data packet contains parameters specific to the node. These must be
		stored in the appropriate table.*/
		NewEvent.NodeShortAddr = CurrPacket->init.header.sourceAddress -2;
		NodesParsArr[NewEvent.NodeShortAddr].Battery = CurrPacket->rfData.Battery;
		NodesParsArr[NewEvent.NodeShortAddr].rssi = CurrPacket->init.preamp.rssi;
		NodesParsArr[NewEvent.NodeShortAddr].MasterCounter = CurrPacket->rfData.MasterCounter;
		NodesParsArr[NewEvent.NodeShortAddr].CALADC12_15V_30C =
			CurrPacket->rfData.CALADC12_15V_30C;
		NodesParsArr[NewEvent.NodeShortAddr].CALADC12_15V_85C =
			CurrPacket->rfData.CALADC12_15V_85C;
		NewEvent.NodeShortAddr += 2;

		/*The PAck is almost ready. Now need to parse the latestRxPacket and store its events
		one by one into the events array, if there is space in it, of course. The events
		stored must be counted and their number should be set in the Answer packet.*/
		AddedStamps = 0;
		for(i = 0; i < CurrPacket->rfData.EventsLen; i++) {
			NewEvent.timestamp = CurrPacket->rfData.Events[i].TimeStamp;
			NewEvent.temperature = CurrPacket->rfData.Events[i].Temperature;
			NewEvent.eventType = CurrPacket->rfData.Events[i].EventType;
			StmpSlot = AddEvent(&NewEvent);
			if(StmpSlot < 0) {				//No more space in the event queue?
				break;						// => Well, stop trying to insert events
			} else if(StmpSlot > 0){		//Stamp added?
				AddedStamps++;				// => Count it in to the added stamps
			}
		}
		AnswerPacket.pack.eventsCount = i;	//The number of accepted events for a PAck
		//Also, keep track of the number of events in the event queue from this node
		NodesParsArr[NewEvent.NodeShortAddr -2].StampsNo += AddedStamps;

		/*if there is more space in the events array, then it means we added all the events
		and we are able to receive more. In that case the packet is converted to an Ack.*/
		if(NodesEvCnt < MAXNODEEVENTS) {
			AnswerPacket.init.header.packetType = RFPacket_Ack;
		}
	}

	//Now we have to send the answering packet
	sendAck(CurrPacket->init.header.sourceAddress);
}


void ParseInfoPacket(union RFPacket* CurrPacket, uint16_t CRCCheck) {
	uint8_t NodeAddr;						//Node's short address from DHCP table
	uint8_t i;								//Helper variable for "for" loops

	//The CRC of the packet must match in order for the packet to be valid
	if(CurrPacket->getInfo.CRC != CRCCheck) {
		return;
	}
	/*The packet contains the node's MAC address. This should be in the DHCP table as the node
	must be registered in order to communicate data to this master node.*/
	NodeAddr = GetNodeAddress(CurrPacket->getInfo.NodeMAC);
	if(NodeAddr == 0) {
		return;
	}
	AnswerPacket.nodeInfo.header.packetType = RFPacket_NodeInfo;
	AnswerPacket.nodeInfo.NodeShortAddr = NodeAddr;
	NodeAddr -= 2;
	NodesParsArr[NodeAddr].rssi = CurrPacket->init.preamp.rssi;
	NodesParsArr[NodeAddr].CALADC12_15V_30C = CurrPacket->getInfo.CALADC12_15V_30C;
	NodesParsArr[NodeAddr].CALADC12_15V_85C = CurrPacket->getInfo.CALADC12_15V_85C;
	NodesParsArr[NodeAddr].MasterCounter = CurrPacket->getInfo.MasterCounter;
	NodesParsArr[NodeAddr].Battery = CurrPacket->getInfo.Battery;
	for(i = 0; i < 8; i++) {
		AnswerPacket.nodeInfo.NodeMAC[i] = CurrPacket->getInfo.NodeMAC[i];
		AnswerPacket.nodeInfo.MasterMAC[i] = beaconPacket.SourceMAC[i];
	}
	AnswerPacket.nodeInfo.flags = NodesParsArr[NodeAddr].flags;
	sendNodeInfo(AnswerPacket.nodeInfo.NodeShortAddr);
}
