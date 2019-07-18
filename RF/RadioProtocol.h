#ifndef RADIOPROTOCOL_H_
#define RADIOPROTOCOL_H_

#include "stdint.h"
#include "RF/EasyLink/EasyLink.h"

#define DEF_BROADCAST_ADDRESS		0x00
#define DEF_MASTER_ADDRESS			0x01
#define DEF_FREQUENCY				868000000
#define DEF_MODULATION				EasyLink_Phy_50kbps2gfsk
#define DEF_RFRESTART				(10 * 60)

/*Ack and PAck packets contain a field named "flags". This field is a kind of data needed to
be sent from the master to the slave node. The following definitions define the meaning of the
bits of this field.*/
#define PF_RESET		(1 << 0)			//Shows the user's will to reset the counters

typedef enum {
	RFPacket_Ack = 1,						// 1
	RFPacket_PAck,							// 2
	RFPacket_NAck,							// 3
	RFPacket_Beacon,						// 4
	RFPacket_Register,						// 5
	RFPacket_RegisterAck,					// 6
	RFPacket_RegisterNAck,					// 7
	RFPacket_GetInfo,						// 8
	RFPacket_NodeInfo,						// 9
	RFPacket_Data							// 10
} RFPacketType;

/*The preamp of a packet is some information that are not transmitted through RF. Due to the
fact that the header of a packet is not a full 32 bit word, the compiler adds dummy bytes
after the header that must be transmitted in order to keep data aligned on a word boundary.
Using the preamp we can use these dummy bytes for useful information and position them before
the packet header, so a normal packet can be sent without adding dummy bytes in it!
	@Params:
 	 	rssi: Is the rssi value from the node.
 	 	flags: Some helpful flags to indicate operations and parameters on the packet.
*/
struct PacketPreamp {
	uint8_t rssi;							//Keeps the RSSI value of the node
	uint8_t len;							//The length of this packet in bytes. The length
											// is the one that the packet has during reception
											// excluding the preamp.
};
#define RFPREAMP_SIZE	2

/*A simple form of the packet. Every RF packet used in this application has a packet header.
It is the first thing transmitted and contains all the common parameters of the packets.
	@Params:
		sourceAddress: The short address of the node that sends the packet
		destAddress: The address of the node that should accept and handle the packet.
		packetType: Is the type of the packet. This type defines the way the packet body is
			parsed and used.
		packetSession: Every complete communication forms a session. There may be more than
			one packets that will comprise the whole communication scheme. All the packets
			that form the communication scheme must have the same session number.
		packetNo: Is the ascending number of the packet in the session.
*/
struct PacketHeader {
//	uint8_t	len;							//Length of the total packet. It is the first
											// entry of EasyLink_TxPacket
//	uint8_t dstAddr[8];						//The destination address. It is included in
											// EasyLink_TxPacket. Keep in mind that though it
											// is 8 bytes long, the address length is by
											// default set to 1. In order to change it,
											// EasyLink_setCtrl(EasyLink_Ctrl_AddSize, <size>)
											// must be called.
	RFPacketType packetType;				//Packet type
	uint8_t sourceAddress;					//Source's short address
};
#define RFHEAD_SIZE		2					//The size of the specified header in bytes

/*The PacketInit is the common denominator of all packet types. Each packet starts with this
structure, so it is a way to access this information whichever packet type it describes.*/
struct PacketInit {
	struct PacketPreamp preamp;
	struct PacketHeader header;
};
#define RFINIT_SIZE		(RFPREAMP_SIZE + RFHEAD_SIZE)

/*The Acknowledge packet, acknowledges that the receiver of the packet has accepted it.
	@Params:
		CRC: is a CRC checksum to check the validity of those data transmitted. The entries
			included in the CRC are all the fields of the packet including its header. The
			only excluded field is the CRC value itself
		packetSession: Every complete communication forms a session. There may be more than
			one packets that will comprise the whole communication scheme. All the packets
			that form the communication scheme must have the same session number.
		packetNo: Is the ascending number of the packet in the session.
		NodeMAC: The MAC address of the node that accepted the packet
*/
struct AckPacket {
	struct PacketPreamp preamp;				//The common to all packets preamp. It is not send
											// through RF but contains helpful data
	struct PacketHeader header;				//The common to all packets header
	uint16_t CRC;							//A CRC of the packet
	uint8_t packetSession;					//Defines the session of the packet that is
											// acknowledged
	uint8_t packetNo;						//Defines the number of the packet in the session
											// that is acknowledged
	uint8_t NodeMAC[8];						//The MAC address of the master node
	uint8_t flags;							//Flags to be sent to the node (for example Reset
											// request from server
};
#define RFACK_SIZE		(RFHEAD_SIZE +13)	//Size of a complete Ack packet
#define RFCRC_SIZE		2					//Size of the CRC entry

/*The Partial Acknowledge packet, acknowledges that the receiver of the packet has accepted
the data packet, but the system was not able to store all the events described in it.
	@Params:
		CRC: is a CRC checksum to check the validity of those data transmitted. The entries
			included in the CRC are all the fields of the packet including its header. The
			only excluded field is the CRC value itself
		packetSession: Every complete communication forms a session. There may be more than
			one packets that will comprise the whole communication scheme. All the packets
			that form the communication scheme must have the same session number.
		packetNo: Is the ascending number of the packet in the session.
		eventsCount: Is the number of events stored in master's memory. The rest could not
			be stored due to lack of memory. The node must retry sending the rest of the
			events in a later communication
*/
struct PAckPacket {
	struct PacketPreamp preamp;				//The common to all packets preamp. It is not send
											// through RF but contains helpful data
	struct PacketHeader header;				//The common to all packets header
	uint16_t CRC;							//A CRC of the packet
	uint8_t packetSession;					//Defines the session of the packet that is
											// acknowledged
	uint8_t packetNo;						//Defines the number of the packet in the session
											// that is acknowledged
	uint8_t NodeMAC[8];						//The MAC address of the master node
	uint8_t flags;							//Flags to be sent to the node (for example Reset
											// request from server
	uint8_t eventsCount;					//Defines the number of events accepted by this
											// packet
};
#define RFPACK_SIZE		(RFHEAD_SIZE +14)	//Size of a complete PAck packet

/*The Not Acknowledge packet, acknowledges that the receiver of the packet has denied it.
NOTE: NAck and Ack packets are totally similar except that the header.packetType value is
different and the NAck packet does not contain any flags.
	@Params:
		CRC: is a CRC checksum to check the validity of those data transmitted. The entries
			included in the CRC are all the fields of the packet including its header. The
			only excluded field is the CRC value itself
		packetSession: Every complete communication forms a session. There may be more than
			one packets that will comprise the whole communication scheme. All the packets
			that form the communication scheme must have the same session number.
		packetNo: Is the ascending number of the packet in the session.
		NodeMAC: The MAC address of the node that accepted the packet
*/
//struct NAckPacket {
//	struct PacketPreamp preamp;				//The common to all packets preamp. It is not send
//											// through RF but contains helpful data
//	struct PacketHeader header;				//The common to all packets header
//	uint16_t CRC;							//A CRC of the packet
//	uint8_t packetSession;					//Defines the session of the packet that is
//											// not acknowledged
//	uint8_t packetNo;						//Defines the number of the packet in the session
//											// that is not acknowledged
//	uint8_t NodeMAC[8];						//The MAC address of the master node
//};
#define RFNACK_SIZE		(RFHEAD_SIZE +12)	//Size of a complete Ack packet

/*The Raw Partial Acknowledge Packet is the real Ack/PAck/NAck packet transmitted through the
RF subsystem. It does not contain a preamp. Its use is to parse an already received Ack, PAck
or NAck packet, as Ack and NAck are subsets of PAck; Ack does not contain the "eventsCount"
field and NAck does not contain "eventsCount" and "flags" fields.
*/
struct RawPAckPacket {
	struct PacketHeader header;				//The common to all packets header
	uint16_t CRC;							//A CRC of the packet
	uint8_t packetSession;					//Defines the session of the packet that is
											// acknowledged
	uint8_t packetNo;						//Defines the number of the packet in the session
											// that is acknowledged
	uint8_t NodeMAC[8];						//The MAC address of the master node
	uint8_t flags;							//Flags to be sent to the node (for example Reset
											// request from server
	uint8_t eventsCount;					//Defines the number of events accepted by this
											// packet
};

/*The Beacon packet, is transmitted by the Master Node to indicate that it can accept packets
from Slave Nodes. It is sent every 100ms. This packet also serves as a RTC synchronizer to the
slave nodes. It does not contain a CRC to preserve time and packet bytes, as it is transmitted
very often and it can be easily identified if there is a fault.
	@Params:
		SourceMAC: Is the MAC address of the master node that sends the packet
		CurrTimeStamp: is the timestamp that defines the current second the beacon is
			transmitted
		EndBeaconTime: is the timestamp that Master node will fall asleep and stop accepting
			any more packets
		NextWakeUpTime: is the timestamp that defines the next wake-up and packet reception
			of this Master node
*/
struct BeaconPacket {
	struct PacketPreamp preamp;				//The common to all packets preamp. It is not send
											// through RF but contains helpful data
	struct PacketHeader header;				//The common to all packets header
	uint32_t CurrTimeStamp;					//The current time in seconds from Jan 1st, 2000
//	uint32_t EndBeaconTime;					//The time that RF activity will stop (timestamp)
	uint32_t NextWakeUpTime;				//The time that next RF activity is going to take
											// place
	uint8_t SourceMAC[8];					//The MAC address of the master pitfall trap that
											// transmits the beacon
};
#define RFBEACON_SIZE	(RFHEAD_SIZE +16)	//Size of a complete Beacon packet

/*The Raw Beacon Packet is the real beacon packet transmitted. It does not contain any preamp
as it is used only for parsing the beacon packet received by the slave node. Besides the
preamp, its contents are identical to the BeaconPacket.*/
struct RawBeaconPacket {
	struct PacketHeader header;				//The common to all packets header
	uint32_t CurrTimeStamp;					//The current time in seconds from Jan 1st, 2000
//	uint32_t EndBeaconTime;					//The time that RF activity will stop (timestamp)
	uint32_t NextWakeUpTime;				//The time that next RF activity is going to take
											// place
	uint8_t SourceMAC[8];					//The MAC address of the master pitfall trap that
											// transmits the beacon
};

/*The Register Slave Node to Master packet. It is used from a slave node to ask a master node
to accept it in its local network. This packet is acknowledged by the master node using a
RegisterAck packet.
	@Params:
		CRC: is a CRC checksum to check the validity of those data transmitted. The entries
			included in the CRC are all the fields of the packet including its header. The
			only excluded field is the CRC value itself
		MasterMAC: is the MAC address of the master that the slave node transmitting this
			request, asks to become a member of its local network. This MAC address is found
			by the slave node from the beacon packet the master transmits
		NodeMAC: is the MAC address of the slave node asking for registration
*/
struct RegisterNodePacket {
	struct PacketPreamp preamp;				//The common to all packets preamp. It is not send
											// through RF but contains helpful data
	struct PacketHeader header;				//The common to all packets header
	uint16_t CRC;							//A CRC of the packet
	uint8_t	MasterMAC[8];					//The MAC address of the Master to bond
	uint8_t NodeMAC[8];						//The MAC address of the node requesting
											// registration
};
#define RFREG_SIZE		(RFHEAD_SIZE +18)	//Size of a complete Registration Request packet

/*The Slave Node Registered Acknowledge packet. It is transmitted by the master node to the
slave one previously asked for registration. It defines the short address of the node and the
registration status (to be used in a later release)
	@Params:
		CRC: is a CRC checksum to check the validity of those data transmitted. The entries
			included in the CRC are all the fields of the packet including its header. The
			only excluded field is the CRC value itself
		NodeMAC: is the MAC address of the node that was accepted by the master
		NodeShortAddr: is the short address that node will be identified (kind of DHCP)
		RegistrationStatus: defines the registration status of the node (something like
			Pending or Accepted). It will be used in a later release were the slave nodes will
			be defined using the host site of the network. Currently it is always 0.
*/
struct RegisterAckPacket {
	struct PacketPreamp preamp;				//The common to all packets preamp. It is not send
											// through RF but contains helpful data
	struct PacketHeader header;				//The common to all packets header
	uint16_t CRC;							//A CRC of the packet
	uint8_t NodeMAC[8];						//The MAC address of the accepted node
	uint8_t NodeShortAddr;					//The short address assigned from master to the
											// registered node
	uint8_t RegistrationStatus;				//The status of registration
};
#define RFREGACK_SIZE	(RFHEAD_SIZE +12)	//Size of a complete Registration Acceptance
											// packet
/*The Raw Registration Acknowledge packet is a helper structure the defines only the
transmitted body of a Registration(N)Ack packet. It is a helper structure to be used for
registration answers from a master, before performing the registration in Flash memory.*/
struct RawRegisterAckPacket {
	struct PacketHeader header;				//The common to all packets header
	uint16_t CRC;							//A CRC of the packet
	uint8_t NodeMAC[8];						//The MAC address of the accepted node
	uint8_t NodeShortAddr;					//The short address assigned from master to the
											// registered node
	uint8_t RegistrationStatus;				//The status of registration
};

/*The Slave Node Registered Deny packet. It is transmitted by the master node to the slave one
previously asked for registration when the master node has denied accepting this node in its
local network.
NOTE: The RegisterNAckPacket is a subset of a RegisterAckPacket. The latter is a superset of
the RegisterNAckPacket, so there is no need to redefine it. By filling in the fields of a
RegisterAckPacket that are common to RegisterNAckPacket and sending that packet with length
equal to RFREGNACK_SIZE, a full RegisterNAckPacket is sent.
	@Params:
		CRC: is a CRC checksum to check the validity of those data transmitted. The entries
			included in the CRC are all the fields of the packet including its header. The
			only excluded field is the CRC value itself
		NodeMAC: is the MAC address of the node that was accepted by the master
*/
//struct RegisterNAckPacket {
//	struct PacketHeader header;				//The common to all packets header
//	uint16_t CRC;							//A CRC of the packet
//	uint8_t NodeMAC[8];						//The MAC address of the denied node
//};
#define RFREGNACK_SIZE	(RFHEAD_SIZE +10)	//Size of a complete Registration Denyance
											// packet
/*Event record defines the record of an event. There are some different kinds of records:
Temperature Only defines a record that does not really contain an event, but used only for
temperature readings, Normal Event Record that defines the time and temperature an event
occured and Reset Event Record defines the time that a Reset event was received from server.
The latter does not contain a real event but only a reset activity.
	@Params:
		TimeStamp: is the timestamp of the event described by the record
		Temperature: is the temperature reading from the sensor of the node
		EventType: is the type of this record (Temperature Only, Normal, Reset)
*/
struct EventRecord {
	uint32_t TimeStamp;						//The timestamp of the event record
	uint16_t Temperature;					//Event temperature
	uint8_t EventType;						//The type of event record:
											// 0: Temperature Only Record
											// 1: Normal Event Record
											// 2: Reset Event Record
	uint8_t EventFlags;						//Flags that describe status of the event
};
#define RFEVENT_SIZE	8					//Size of an event record structure

/*The Data packet is transmitted by a slave node to send event records to the associated
Master one. The maximum number of events that can be sent to the master node is defined by
MAXEVENTS.
	@Params:
		CRC: is a CRC checksum to check the validity of those data transmitted. The entries
			included in the CRC are all the fields of the packet including its header. The
			only excluded field is the CRC value itself
		packetSession: Every complete communication forms a session. There may be more than
			one packets that will comprise the whole communication scheme. All the packets
			that form the communication scheme must have the same session number.
		packetNo: Is the ascending number of the packet in the session.
		EventsLen: is the number of event structures included in this packet
		Battery: is the ADC value of the battery voltage measurement
		MasterCounter: is the total number of events the node has recorded since its last
			reset
		Events: is the array of event records transmitted from slave node to its master
Some macros are also defined. Since the real size of the packet depends on the number of
events that appear in the Events array, there are some macros defined to help calculating the
length of the packet in bytes.
MAXEVENTS defines the maximum number of events that can be in one packet.
RFDATA_SIZE(x) returns the number of bytes the packet contains for x events in Events array
RFDATA_MAXSIZE returns the maximum number of bytes when using the maximum number of events in
	the packet.
*/
//Maximum number of events in a packet
#define MAXEVENTS 		((EASYLINK_MAX_DATA_LENGTH -16 -RFHEAD_SIZE) /RFEVENT_SIZE)
struct DataPacket {
	struct PacketPreamp preamp;				//The common to all packets preamp. It is not send
											// through RF but contains helpful data
	struct PacketHeader header;				//The common to all packets header
	uint16_t CRC;							//A CRC of the packet
	uint16_t CALADC12_15V_30C;				//Temperature Sensor Calibration value for 30C
	uint16_t CALADC12_15V_85C;				//Temperature Sensor Calibration value for 85C
	uint16_t Battery;						//The battery ADC value
	uint16_t MasterCounter;					//The master counter of the node's events
	uint16_t GSMCounter;					//This field is always 0 as it describes the
											// number of events while GSM was on. Nodes do not
											// have a GSM module. It is needed to patch a half
											// word in the packet in order to keep Events
											// table 32 bit aligned.
	uint16_t EventsLen;						//The length of the following array of events
											// eliaschr@NOTE: This parameter can be 8 bits,
											// but it is used to align the Events Record array
											// to 32-bit word
	uint8_t packetSession;					//Defines the session of the packet that is
											// acknowledged
	uint8_t packetNo;						//Defines the number of the packet in the session
											// that is acknowledged
	struct EventRecord Events[MAXEVENTS];	//Events array
};
#define RFDATA_MAXSIZE	(RFHEAD_SIZE +16 +(MAXEVENTS *RFEVENT_SIZE))
#define RFDATA_SIZE(x)	(RFHEAD_SIZE +16 +(x *RFEVENT_SIZE))

/*The Get Node Information packet. It is transmitted by the slave node to the master when it
needs to get information about itself from the node. The master must answer using a Node Info
packet (described later).
	@Params:
		CRC: is a CRC checksum to check the validity of those data transmitted. The entries
			included in the CRC are all the fields of the packet including its header. The
			only excluded field is the CRC value itself
		NodeMAC: is the MAC address of the node that requests information from the master
*/
struct GetInfoPacket {
	struct PacketPreamp preamp;				//The common to all packets preamp. It is not send
											// through RF but contains helpful data
	struct PacketHeader header;				//The common to all packets header
	uint16_t CRC;							//A CRC of the packet
	uint8_t NodeMAC[8];						//The MAC address of the node requesting info
	uint16_t CALADC12_15V_30C;				//Temperature Sensor Calibration value for 30C
	uint16_t CALADC12_15V_85C;				//Temperature Sensor Calibration value for 85C
	uint16_t Battery;						//The battery ADC value
	uint16_t MasterCounter;					//The master counter of the node's events
};
#define RFGETINFO_SIZE	(RFHEAD_SIZE +18)	//Size of a raw GetInfo packet in bytes

/*The Node Information packet. It is transmitted by the master node to the slave one
previously asked for information, through a Get Node Information packet. It returns the node's
short address assigned by the master and the flags that describe the parameters sent by the
internet server.
	@Params:
		CRC: is a CRC checksum to check the validity of those data transmitted. The entries
			included in the CRC are all the fields of the packet including its header. The
			only excluded field is the CRC value itself
		MasterMAC: is the MAC address of the master node that announces to the slave node its
			recorded parameters
		NodeMAC: is the MAC address of the node that the master directs the answer to
		NodeShortAddr: is the short address assigned from the master to that node
		flags: Is a byte that transfers parameters from the internet server, such as "RESET"
*/
struct NodeInfoPacket {
	struct PacketPreamp preamp;				//The common to all packets preamp. It is not send
											// through RF but contains helpful data
	struct PacketHeader header;				//The common to all packets header
	uint16_t CRC;							//A CRC of the packet
	uint8_t NodeMAC[8];						//The MAC address of the node in question
	uint8_t MasterMAC[8];					//The MAC address of the master node
	uint8_t NodeShortAddr;					//The short address of that node
	uint8_t flags;							//Flags that come from the server
	uint16_t RadioTm;						//Restarting RF activity interval
};
#define RFNODEINFO_SIZE	(RFHEAD_SIZE +22)	//Size of a raw NodeInfo packet in bytes

/*The raw Node Information Packet that contains all the data a NodeInfoPacket contains except
the preamp. In that way the code can access an already received packet and get its data in a
simple manner. It can be used as a raw Get Information Packet but the fields after NodeMAC
will be invalid (not in use in a GetInfo packet).*/
struct RawInfoPacket {
	struct PacketHeader header;				//The common to all packets header
	uint16_t CRC;							//A CRC of the packet
	uint8_t NodeMAC[8];						//The MAC address of the node in question
	uint8_t MasterMAC[8];					//The MAC address of the master node
	uint8_t NodeShortAddr;					//The short address of that node
	uint8_t flags;							//Flags that come from the server
	uint16_t RadioTm;						//Restarting RF activity interval
};

#endif /* RADIOPROTOCOL_H_ */
