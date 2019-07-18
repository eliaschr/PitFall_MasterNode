#ifndef TASKS_MAINTASK_H_
#define TASKS_MAINTASK_H_

#include <stdint.h>
#include <ti/sysbios/knl/Clock.h>

#define MAXNODEEVENTS		128
#define MAXNODES			254

typedef struct {
	uint32_t timestamp;
	uint16_t temperature;
	uint8_t eventType;
	uint8_t NodeShortAddr;
} NodeEvent;
#define NODEEVENT_SIZE		8

typedef struct {
	uint16_t CALADC12_15V_30C;
	uint16_t CALADC12_15V_85C;
	uint16_t MasterCounter;
	uint16_t Battery;
	uint8_t SndTm;
	uint8_t StampsNo;
	uint8_t rssi;
	uint8_t flags;
} NodePars;
#define NODEPARS_SIZE		12

#define NPF_GSMSYNCED		(1<<0)			//Shows if there has been any GPRS Settings
											// for this node
#define NPF_GSMRESET		(1<<1)			//Shows if there is a request to reset node's
											// counters

extern Clock_Handle ReGSMClk;				//Clock handle for GSM task restarting
extern Clock_Handle ReRFClk;				//Clock handle for RF task restarting
extern NodeEvent EvArray[MAXNODEEVENTS];	//Events queue array. Stores the events sent from
											// slave nodes
extern NodePars NodesParsArr[MAXNODES];		//Variable parameters of every node
extern uint16_t NodesEvCnt;					//Events stored in Node Events Array
extern uint16_t VBattLvl;					//The battery voltage level of the master node

void MainTask_init(void);					//Initialize the main task
int16_t AddEvent(NodeEvent* InEv);			//Add an event in the events array

#endif /* TASKS_MAINTASK_H_ */
