/*********************************************************************************************
* PitFall_MasterNode.h                                                                       *
**********************************************************************************************
*  Created on: Sep 26, 2016                                                                  *
*      Author: eliaschr                                                                      *
**********************************************************************************************
*/

#ifndef PITFALL_MASTERNODE_H_
#define PITFALL_MASTERNODE_H_

#include <stdint.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/drivers/PIN.h>

#define GPRSEV_ERROR		Event_Id_00
#define GPRSEV_COMMOK		Event_Id_01
#define GPRSEV_ALL			(GPRSEV_ERROR | GPRSEV_COMMOK)

#define RFEV_DONE			Event_Id_16
#define RFEV_NEWDATA		Event_Id_17
#define RFEV_NEWNODE		Event_Id_18
#define RFEV_ALL			(RFEV_DONE | RFEV_NEWNODE | RFEV_NEWDATA)

/* Type definitions */
typedef void (*CallbackFunc)(void);

extern Event_Handle MainEvHandle;
extern PIN_Handle ledPinHandle;
extern PIN_Handle swPinHandle;

void Halt_abort(char* ErrorTxt, void* ptr);
void registerButtonCb(PIN_Id pinId, CallbackFunc cbFxn);

#endif /* PITFALL_MASTERNODE_H_ */
