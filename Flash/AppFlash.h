/*********************************************************************************************
 * AppFlash.h                                                                                *
 *===========================================================================================*
 *  Created on: Aug 28, 2016                                                                 *
 *      Author: eliaschr                                                                     *
 ********************************************************************************************/

#ifndef APPFLASH_H_
#define APPFLASH_H_

/*********************************************************************************************
 * Includes                                                                                  *
 ********************************************************************************************/


/*********************************************************************************************
 * Definitions                                                                               *
 ********************************************************************************************/
/*The internet connection needs some parameters. Their default values are here. Some of them
are overloaded by settings in Flash memory*/
#define	DEFHOST			"www.rhynchos.com"	//Default server address
#define DEFUID			"866771029055752"	//Default IMEI of the device
#define DEFAPN			"gint.b-online.gr"	//Default APN
#define DEFPORT			"80"				//Default port to be used for server (http port)
#define DEFUSERNAME		""					//Default username for GPRS communication
#define DEFPASSWORD		""					//Default password for GPRS communication
#define DEFRTCCAL		0x00800000			//Default value of AON_RTC_O_SUBSECINC
#define DEFNODTM		10					//Default update time for RF Nodes in minutes


/*********************************************************************************************
 * Flash Memory DHCP Service Area                                                            *
 ********************************************************************************************/
/*There is a table of accepted MAC addresses in the Flash memory. This is used to correlate
MAC addresses to short addresses and to add a level of security for not accepting data from
unauthorized nodes. This table is described in the following part. The total length of this
table is 3KB = 3072 Bytes.*/
typedef struct {
	uint32_t Flags;							//Flags for Node, default value is 0xFFFFFFFF
	uint8_t MACAddr[8];						//MAC address of a node
} DHCPEntry;								//Structure of a DHCP entry

#define BASE_DHCP		0x0001D000			//Starting offset of Nodes' DHCP Table
#define DHCPENTRY_LEN	12					//Length of a DHCP entry in bytes
#define FLAGSOFFSET		0					//Offset of Flags in a DHCP entry
#define MACOFFSET		4					//Offset of MAC address in a DHCP entry
#define MACOFFSETLO		4					//Offset of MAC address low word in a DHCP entry
#define MACOFFSETHI		8					//Offset of MAC address high word in a DHCP entry
#define DEFFLAGS		0xFFFFFFFF			//Default flags. All flags are reserved (set)

//Default DHCP ID - 32 bits at the top of this segment
#define DEFDHCPID		0xEC1CBCAC					//DHCP ID value for validity checking
#define DHCPIDOFFSET	0x0FFC						//Offset of DHCP ID value in segment
#define DHCPIDLENGTH	4							//Maximum length of DHCP ID
#define DhcpID			((int32_t*) (BASE_DHCP + DHCPIDOFFSET))//Physical address of DHCP ID


/*********************************************************************************************
 * Flash Memory Configuration Addresses                                                      *
 ********************************************************************************************/
#define BASE_GSM		0x0001E000					//Offset of memory the GSM settings are
													// stored
//Default APN - 64 characters
#define APNOFFSET		0							//Offset of APN value in segment
#define APNLENGTH		64							//Maximum length of APN value
#define APN				((char*) (BASE_GSM + APNOFFSET))	//Physical address of APN

//Default username for GPRS communication - 32 chr
#define USEROFFSET		(APNOFFSET + APNLENGTH)		//Offset of Username value in segment
#define USERLENGTH		32							//Maximum length of Username value
#define Username		((char*) (BASE_GSM + USEROFFSET))	//Physical address of Username

//Default password for GPRS communication - 32 chr
#define PASSOFFSET		(USEROFFSET + USERLENGTH)	//Offset of Password value in segment
#define PASSLENGTH		32							//Maximum length od Password value
#define Password		((char*) (BASE_GSM + PASSOFFSET))	//Physical address of Password

//Default server address - 32 chrs
#define HOSTOFFSET		(PASSOFFSET + PASSLENGTH)	//Offset of Host address value in segment
#define HOSTLENGTH		32							//Maximum length of Host value
#define Host			((char*) (BASE_GSM + HOSTOFFSET))	//Physical address of Host

//Default http port to be used for server - 6 chr (ASCII)
#define PORTOFFSET		(HOSTOFFSET + HOSTLENGTH)	//Offset of server's Port in segment
#define PORTLENGTH		8							//Maximum length of Port value
#define Port			((char*) (BASE_GSM + PORTOFFSET))	//Physical address of Port

//Default IMEI of the device - 16 chrs
#define UIDOFFSET		(PORTOFFSET + PORTLENGTH)	//Offset of UID (IMEI) value in segment
#define UIDLENGTH		16							//Maximum length of UID in characters
#define UID				((char*) (BASE_GSM + UIDOFFSET))	//Physical address of UID

//Default RTCCAL of the device - 32 bits
#define RTCOFFSET		(UIDOFFSET + UIDLENGTH)		//Offset of RTCCAL value in segment
#define RTCLENGTH		4							//Maximum length of RTCCAL in bytes
#define RTCCal			((uint32_t*) (BASE_GSM + RTCOFFSET))//Physical address of UID

//Default Node Update Time in minutes - 16 bits
#define NODTMOFFSET		(RTCOFFSET + RTCLENGTH)		//Offset of NODTM value in segment
#define NODTMLENGTH		2							//Maximum length of NODTM in bytes
#define NodTm			((uint16_t*) (BASE_GSM + NODTMOFFSET))//Physical address of NODTM

#define VARSLENGTH		(NODTMOFFSET + NODTMLENGTH)	//The total length of the variables area

//Default Author ID - 32 bits at the top of this segment
#define AID				0xEC1CBCAC					//Author ID value for validity checking
#define AIDOFFSET		0xFC						//Offset of Author ID value in segment
#define AIDLENGTH		4							//Maximum length of Author ID
#define AuthorID		((int32_t*) (BASE_GSM + AIDOFFSET))//Physical address of Author ID

#if ((RTCOFFSET & 0x03) != 0)
#error "RTCOFFSET is not 32 bit aligned!"
#endif

#if ((AIDOFFSET & 0x03) != 0)
#error "AIDOFFSET is not 32 bit aligned!"
#endif

#if ((NODTMOFFSET & 0x01) != 0)
#error "NODTMOFFSET is not 16 bit aligned!"
#endif


/*********************************************************************************************
 * Definitions of new types                                                                  *
 ********************************************************************************************/
typedef union {
	char cBuf[VARSLENGTH];					//FlashBuffer in RAM to use for Flash programming
	uint8_t ui8Buf[VARSLENGTH];				//Accessing it as 8 bits unsigned integer
	uint16_t ui16Buf[VARSLENGTH/2];			//Accessing it as 16 bit unsigned integer
	uint32_t ui32Buf[VARSLENGTH/4];			//Accessing it as 32 bits unsigned integer
} FlUniv;


/*********************************************************************************************
 * Function declarations, functions other code needs to know about                           *
 ********************************************************************************************/
uint32_t InitFlash(void);					//Initializes the flash area with the defaults
uint32_t StoreUID(char* InUID);				//Stores a new UID in Flash Memory GSM variables
uint32_t StoreParams(uint32_t SMSStatus);	//Stores the fetched parameters in Flash memory

uint8_t AddNode(uint8_t *InMAC);			//Adds a node to DHCP table and returns
											// its short address
DHCPEntry* GetNodeEntry(uint8_t ShortAddress);	//Get a DHCP entry from the Short address of
											// the node
uint8_t GetNodeAddress(uint8_t* InMAC);		//Gets the short address of a node from DHCP table

/*********************************************************************************************
 * Variable declarations, variables other code needs to know about                           *
 ********************************************************************************************/
extern FlUniv FlRAM;
extern uint8_t NodesCnt;

#endif /* APPFLASH_H_ */
