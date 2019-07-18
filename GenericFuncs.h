/********************************************************************************************
 * GenericFuncs.h                                                                           *
 *==========================================================================================*
 *  Created on: Aug 28, 2016                                                                *
 *      Author: eliaschr                                                                    *
 *******************************************************************************************/

#ifndef GENERICFUNCS_H_
#define GENERICFUNCS_H_

/********************************************************************************************
 * Includes                                                                                 *
 *******************************************************************************************/
#include <stdint.h>


/********************************************************************************************
 * Definitions                                                                              *
 *******************************************************************************************/
#define DEF_CRC16	0xFFFF					//Default value of CRC16 checksum at the first
											// byte
#define CRC16_POLY	0x8005					//Defines the polynomial used for CRC16 calcu-
											// lation.

/********************************************************************************************
 * Function declarations, functions other code needs to know about                          *
 *******************************************************************************************/
int8_t ParseNumber(char* InStr, uint16_t* OutVal);	//Parses a string as a Bin, Hex or Dec
int8_t ParseNumber32(char* InStr, uint32_t* OutVal);//Parses a string as a Bin, Hex or Dec
int16_t SkipCRLF(char* InBuffer, int16_t InSize);	//Counts the CRLF characters in InBuffer
int16_t FindCRLF(char* InBuffer, int16_t InSize);	//Counts the number of valid characters
													// until the first CR or LF character
int16_t Hex2Int(char *InVal, uint8_t Len);			//Converts ASCII Hex number to integer
void Byte2Hex(uint8_t InVal, char *OutStr);		//Converts byte value to ASCII Hex string
int16_t Int2Ascii(int16_t InVal, char* Buffer, int16_t BufLen);
													//Converts and integer number into an
													// ASCIIZ representation in a buffer
uint16_t StrSize(char* InStr);						//Finds the length of the input string
uint16_t CalcCRC16(uint8_t InData, uint16_t OldCRC);//Calculates CRC16 of data

#endif /* GENERICFUNCS_H_ */
