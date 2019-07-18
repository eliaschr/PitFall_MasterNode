#ifndef TASKS_RFTASK_H_
#define TASKS_RFTASK_H_

#include "stdint.h"
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>
#include "RF/RadioProtocol.h"

#define PACKETQ_SIZE		8
#define ACKQ_SIZE			64				//Size of the expected Ack packets from nodes

#define RF_EVENT_START		Event_Id_00

#define RFBEACON_TIMEOUT	(0.250 *1000000/Clock_tickPeriod)
#define RFSCAN_TIMEOUT		(30 *1000000/Clock_tickPeriod)
#define RFFLASH_TIMEOUT		(0.3 *1000000/Clock_tickPeriod)
#define RFACK_TIMEOUT		(0.16 *1000000/Clock_tickPeriod)
#define RFLONGKEY_TIMEOUT	(3 *1000000/Clock_tickPeriod)

#define RFBEACON_DURATION	(2 *60)			//Duration of RF activity, in seconds
#define INHIBITGSM_TIMEOUT	180				//Time that GSM is inhibited, before RF
											// communication starts, in seconds

#define RFS_FOUND			(1 << 30)		//Used while searching
#define RFS_NEEDGPRS		(1 << 5)		//GPRS should start just after RF task ends
#define RFS_DELAYGPRS		(1 << 4)		//GPRS firing should be delayed
#define RFS_EXPECTACK		(1 << 3)		//RF task expects an Ack from a node
#define RFS_REGISTER		(1 << 2)		//RF task is ready to register new node
#define RFS_SYNC			(1 << 1)		//RF syncs new nodes
#define RFS_ONAIR			(1 << 0)		//RF communication is activated


enum RFStatus {
    RFStatus_Success,
    RFStatus_Failed,
    RFStatus_FailedNotConnected,
};

typedef struct {
	uint32_t timeTicks;
	uint8_t NodeAddr;
	uint8_t packetSession;
	uint8_t packetNo;
	uint8_t flags;
} AckBack;

extern volatile uint32_t RFStatusFlags;
extern uint32_t NextWakeUpTime;
extern Event_Handle RFEventHandle;
extern union RFPacket PacketQ[PACKETQ_SIZE];
extern Clock_Handle RFTimeoutClk;
extern volatile uint8_t PacketQFirst;
extern volatile uint8_t PacketQLen;
extern AckBack AckQueue[ACKQ_SIZE];
extern volatile uint8_t AckQStrt;
extern volatile uint8_t AckQLen;
extern volatile uint8_t AckQClk;

union RFPacket {
	uint8_t raw[EASYLINK_MAX_DATA_LENGTH];	//To handle a packet as raw bytes
    struct PacketInit init;					//Not a real packet. It is the common denominator
    										// of all described packets
	struct AckPacket ack;
	struct PAckPacket pack;
//	struct NAckPacket nack;					//NAck packet is a Ack packet with different type
	struct BeaconPacket beacon;
	struct RegisterNodePacket regNode;
	struct RegisterAckPacket regAck;
//	struct RegisterNAckPacket regNAck;		//RegisterNAck packet is similar to RegisterAck
	struct DataPacket rfData;
	struct GetInfoPacket getInfo;
	struct NodeInfoPacket nodeInfo;
};

typedef void (*RFPacketCb)(union RFPacket* packet, int8_t rssi);

/* Create the RadioTask and all associated TI-RTOS objects */
void InitRFTask(void);

/* Register the packet received callback */
//void RFTask_registerPacketCb(RFPacketCb callback);
void ParseRegPacket(union RFPacket* CurrPacket, uint16_t CRCCheck);
void ParseDataPacket(union RFPacket* CurrPacket, uint16_t CRCCheck);
void ParseInfoPacket(union RFPacket* CurrPacket, uint16_t CRCCheck);
void RFRestart(UArg InArg);					//Restarts RF activity

#endif /* TASKS_RFTASK_H_ */
