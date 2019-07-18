/********************************************************************************************
 * AppFlash.c                                                                               *
 *==========================================================================================*
 *  Created on: Aug 28, 2016                                                                *
 *      Author: eliaschr                                                                    *
 *******************************************************************************************/

/********************************************************************************************
 * Includes                                                                                 *
 *******************************************************************************************/
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>

/* TI-RTOS Driver files */
#include <driverlib/flash.h>

/* Application Header files */
#include <string.h>
#include "Flash/AppFlash.h"
#include "GSM/UART_GPRS.h"

/********************************************************************************************
 * Definitions to control the whole code behaviour                                          *
 *******************************************************************************************/


/********************************************************************************************
 * Constants definitions                                                                    *
 *******************************************************************************************/


/********************************************************************************************
 * Variable definitions                                                                     *
 *******************************************************************************************/
#pragma DATA_ALIGN(FlRAM, 4)
FlUniv FlRAM;
uint8_t NodesCnt;

/********************************************************************************************
 * Function declarations                                                                    *
 *******************************************************************************************/


/********************************************************************************************
 * Main Flash Handling code                                                                 *
 *******************************************************************************************/

//*******************************************************************************************
/* Initializes all the necessary variables in Flash memory if they are not already. The
variables in question lie in Information memory. There is one ID variable that if the Flash
memory is initialized, it contains valid ID. If not, the memory is re-written with default
data
*/
uint32_t InitFlash(void) {
	uint32_t result;						//Holds the result of Flash operations
	uint32_t MACLo;							//Holds the lower half of a MAC Address
	uint32_t MACHi;							//Holds the higher half of a MAC address

	/*FlashProgram and FlashSectorErase must be called from ROM or RAM and not from Flash
	memory. The same is true for their utilization variables (FlashProgram). During Flash
	programming there cannot be any read access to Flash memory, so we need a buffer in RAM
	to hold the data to be stored in Flash.*/

	//Lets start with the DHCP table, to count the number of registered nodes
	NodesCnt = 0;							//Assume DHCP table is empty
	if(*DhcpID == DEFDHCPID) {				//Initialized DHCP table?
		//Yes => Lets count the number of registered nodes
		result = BASE_DHCP +MACOFFSET;		//Points to the first MAC address
		for(NodesCnt = 0; NodesCnt < 254; NodesCnt++) {
			MACLo = *((uint32_t*)result);	//Get the lower word of this MAC address
			MACHi =  *((uint32_t*)result +4);//Get the higher word of this MAC address
			if((MACLo == 0xFFFFFFFF) && (MACHi == 0xFFFFFFFF)) {
				break;						//If the entry is empty, then exit loop
			}
			result += DHCPENTRY_LEN;		//Point to next entry
		}
		//Now NodesCnt contains the number of registered nodes.
	} else {
		//No => Initialize it
		result = FlashSectorErase(BASE_DHCP);	//Erase the sector of the DHCP table
		if(result != FAPI_STATUS_SUCCESS) {
			return result;						//Cannot erase the sector...
		}
		//Need to write the ID to verify the DHCP table is initialized
		FlRAM.ui32Buf[0] = DEFDHCPID;			//Store the ID into RAM
		return FlashProgram(FlRAM.ui8Buf, BASE_DHCP + DHCPIDOFFSET, DHCPIDLENGTH);
	}

	//Proceed with the GSM and RF communication variables
	if(*AuthorID == AID) {					//If the flash memory is initialized, just reset
											// the RAM image of flash configuration
		for(result = 0; result <VARSLENGTH; result++) {
			FlRAM.ui8Buf[result] = 0xFF;
		}
		return FAPI_STATUS_SUCCESS;
	}

	//If we reach this point, variables' flash memory is not initialized!
	result = FlashSectorErase(BASE_GSM);	//Erase the sector of the GSM non volatile vars
	if(result != FAPI_STATUS_SUCCESS) {
		return result;						//Cannot erase the sector of GSM variables...
	}

	/*Prepare the temporary RAM buffer to hold the data to be written. The default byte value
	of each memory cell is FFh*/
	for(result = 0; result < VARSLENGTH; result++) {
		FlRAM.ui8Buf[result] = 0xFF;
	}
	/*For every parameter we need to calculate the minimum number of bytes between the size
	of the parameter in question and the memory space available for it. We use "result"
	variable as a helper for this calculation, where needed.*/
	result = sizeof(DEFAPN);				//Going to setup APN
	strncpy(&FlRAM.cBuf[APNOFFSET], DEFAPN, (result > APNLENGTH)?APNLENGTH:result);
	result = sizeof(DEFUSERNAME);			//Going to setup Username
	strncpy(&FlRAM.cBuf[USEROFFSET], DEFUSERNAME, (result > USERLENGTH)?USERLENGTH:result);
	result = sizeof(DEFPASSWORD);			//Going to setup Password
	strncpy(&FlRAM.cBuf[PASSOFFSET], DEFPASSWORD, (result > PASSLENGTH)?PASSLENGTH:result);
	result = sizeof(DEFHOST);				//Going to setup Host url
	strncpy(&FlRAM.cBuf[HOSTOFFSET], DEFHOST, (result > HOSTLENGTH)?HOSTLENGTH:result);
	result = sizeof(DEFPORT);				//Going to setup Host port
	strncpy(&FlRAM.cBuf[PORTOFFSET], DEFPORT, (result > PORTLENGTH)?PORTLENGTH:result);
	FlRAM.ui32Buf[RTCOFFSET/4] = DEFRTCCAL;	//Store the default clock running calibration val.
	FlRAM.ui16Buf[NODTMOFFSET/2] = DEFNODTM;//and the default Nodes Updating Interval
	//Flash those data in the desired position
	result = FlashProgram(FlRAM.ui8Buf, BASE_GSM, VARSLENGTH);
	if(result != FAPI_STATUS_SUCCESS) {
		return result;						//Cannot write data? return with error code
	}
	FlRAM.ui32Buf[0] = AID;					//Store the data into RAM
	return FlashProgram(FlRAM.ui8Buf, BASE_GSM + AIDOFFSET, AIDLENGTH);
}


//********************************************************************************************
/*Stores the UID variable in Flash Memory*/
uint32_t StoreUID(char* InUID)
{
	int i;
	char TempChr = 0xFF;

	//First check if UID flash memory is cleared
	for(i = 0; (i < UIDLENGTH) && (TempChr == 0xFF); i++) {
		TempChr &= UID[i];
	}
	if(TempChr != 0xFF) {					//Not cleared? => Need to use a temporary storage
		for(i = 0; i < VARSLENGTH; i++) {	//Copy all bytes of the GSM variables' segment...
			FlRAM.cBuf[i] = APN[i];			//... to RAM Buffer
		}
		for(i = 0; i < UIDLENGTH; i++) {	//Must reset all caracters in UID field
			FlRAM.cBuf[UIDOFFSET +i] = 0xFF;//Reset this chracter in UID
		}
		i = FlashSectorErase(BASE_GSM);		//Erase the sector of the GSM non volatile vars
		if(i != FAPI_STATUS_SUCCESS) {		//Problems erasing the segment?
			return i;						//Yes => Return the status
		}
	}
	i = sizeof(InUID);						//Get the size of the willing UID and copy it to
	strncmp(&FlRAM.cBuf[UIDOFFSET], InUID, (i > UIDLENGTH)?UIDLENGTH:i);	// RAM buffer
	if(TempChr != 0xFF) {					//Do we have to program the whole memory segment?
		i = FlashProgram(FlRAM.ui8Buf, BASE_GSM, VARSLENGTH);	//Yes => Program all vars.
	} else {
		//No => Program only new UID
		i = FlashProgram((uint8_t *)InUID, (uint32_t)UID, UIDLENGTH);
	}
	if(i != FAPI_STATUS_SUCCESS) {			//Problems programming the segment?
		return i;							//Yes => Return the status
	}
	//Last but not least, we must reprogram the AuthorID or else the system will consider the
	// contents of the flash memory invalid
	FlRAM.ui32Buf[0] = AID;					//Store the AuthorID into RAM
	return FlashProgram(FlRAM.ui8Buf, BASE_GSM + AIDOFFSET, AIDLENGTH);
}


//********************************************************************************************
/*Checks all the received parameters from SMS messages and stores all of them to flash memory.
*/
uint32_t StoreParams(uint32_t SMSStatus) {
	uint32_t result;						//Holds the result of Flash operations
	//Do we really need to store something in Flash memory?
	if((SMSStatus & SMS_FLASHMASK) != 0) {	//Yes, we have something to be saved, so...
		//Lets prepare Flash buffer variable to contain the old data where necessary
		if((SMSStatus & SMS_APNSET) == 0) {	//Do we have to copy APN?
			//No => Copy the old one to RAM image
			for(result = 0; result < APNLENGTH; result++) {
				FlRAM.cBuf[APNOFFSET +result] = APN[result];
			}
		}
		if((SMSStatus & SMS_USERSET) == 0) {//Do we have to copy Username?
			//No => Copy the old one to RAM image
			for(result = 0; result < USERLENGTH; result++) {
				FlRAM.cBuf[USEROFFSET +result] = Username[result];
			}
		}
		if((SMSStatus & SMS_PASSSET) == 0) {//Do we have to copy Password?
			//No => Copy the old one to RAM image
			for(result = 0; result < PASSLENGTH; result++) {
				FlRAM.cBuf[PASSOFFSET +result] = Password[result];
			}
		}
		if((SMSStatus & SMS_HOSTSET) == 0) {//Do we have to copy Host?
			//No => Copy the old one to RAM image
			for(result = 0; result < HOSTLENGTH; result++) {
				FlRAM.cBuf[HOSTOFFSET +result] = Host[result];
			}
		}
		if((SMSStatus & SMS_PORTSET) == 0) {//Do we have to copy Port?
			//No => Copy the old one to RAM image
			for(result = 0; result < PORTLENGTH; result++) {
				FlRAM.cBuf[PORTOFFSET +result] = Port[result];
			}
		}
		if((SMSStatus & SMS_UIDSET) == 0) {	//Do we have to copy UID?
			//No => Copy the old one to RAM image
			for(result = 0; result < UIDLENGTH; result++) {
				FlRAM.cBuf[UIDOFFSET +result] = UID[result];
			}
		}
		if((SMSStatus & SMS_CALSET) == 0) {	//Do we have to copy RTC Calibration value?
			//No => Copy the old one to RAM image
			FlRAM.ui32Buf[RTCOFFSET/4] = *RTCCal;
		}
		if((SMSStatus & SMS_NODTMSET) == 0) {	//Do we have to copy Node Update Time?
			//No => Copy the old one to RAM image
			FlRAM.ui16Buf[NODTMOFFSET/2] = *NodTm;
		}
		//Erase the sector of the GSM non volatile vars
		result = FlashSectorErase(BASE_GSM);
		if(result != FAPI_STATUS_SUCCESS) {
			return result;					//Cannot erase the sector of GSM variables...
		}
		//Program the new data
		result = FlashProgram(FlRAM.ui8Buf, BASE_GSM, VARSLENGTH);
		if(result != FAPI_STATUS_SUCCESS) {
			return result;					//Cannot write new data? => return with error
		}
		/*Last but not least, we must reprogram the AuthorID or else the system will consider
		the contents of the flash memory invalid.*/
		FlRAM.ui32Buf[0] = AID;				//Store the AuthorID into RAM
		return FlashProgram(FlRAM.ui8Buf, BASE_GSM + AIDOFFSET, AIDLENGTH);
	}
	return FAPI_STATUS_SUCCESS;
}


//********************************************************************************************
/*Add a node into DHCP table and return its short address. The DHCP table is in Flash memory
and contains all the MAC addresses of the registered nodes. The position of the node in this
table (index of entry) defines its short address. The first node's short address is 2.
In every entry there is a 32 bit field called flags. This is reserved for future use, where
there can be nodes' management through the PitFall traps internet host page.
If the writing is successful then the function returns the short address (entry index +2). If
it is not, it returns 0 which is the broadcast address and of course not a valid one for a
single node.
*/
uint8_t AddNode(uint8_t *InMAC) {
	uint32_t EntryPtr;						//Pointer to an entry
	uint8_t result;							//Resulting short address

	if(NodesCnt >= 254) {					//No more nodes allowed?
		return 0;							//Exit with broadcast address
	}

	result = GetNodeAddress(InMAC);			//Is this node already registered?
	if(result != 0) {						//If found =>
		return result;						// just return its short address
	}

	//Find the address of the first empty DHCP entry.
	EntryPtr = BASE_DHCP + (NodesCnt * DHCPENTRY_LEN);
	int i = FlashProgram(InMAC, EntryPtr +MACOFFSET, 8);
	if(i != FAPI_STATUS_SUCCESS) {
		return 0;							//Error while writing => exit with address 0
	}
	//Writing was successful. Another node in the table
	NodesCnt++;								//Add the new node in the number of available ones
	return (NodesCnt +1);					//Return the node's short address
}


//********************************************************************************************
/*Returns a pointer to the DHCP Entry that belongs to the node with short ID equal to input
parameter ShortAddress. If there is no node with that short address then it returns NULL.
*/
DHCPEntry* GetNodeEntry(uint8_t ShortAddress) {
	uint32_t EntryPtr;

	if((ShortAddress -1) > NodesCnt) {
		return NULL;
	}
	EntryPtr = BASE_DHCP + ((ShortAddress -2) * DHCPENTRY_LEN);
	return (DHCPEntry*)EntryPtr;
}


//********************************************************************************************
/*Returns the Short Address of the node with the MAC address pointed by the input parameter.
If the MAC address is not found then it returns 0, which is the broadcast address.
*/
uint8_t GetNodeAddress(uint8_t* InMAC) {
	uint32_t EntryPtr;						//Pointer to current DHCP entry
	uint8_t i, j;							//Helper loop variables
	uint8_t result = 0;						//The resulting address

	EntryPtr = BASE_DHCP + MACOFFSET;
	for(i = 0; i < NodesCnt; i++) {
		for(j = 0; (j < 8) && (*((uint8_t*)(EntryPtr +j)) == InMAC[j]); j++);
		if(j == 8) {
			result = i +2;
			break;
		}
		EntryPtr += DHCPENTRY_LEN;
	}
	return result;
}
