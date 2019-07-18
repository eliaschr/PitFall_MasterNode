/********************************************************************************************
 * GenericFuncs.c                                                                           *
 *==========================================================================================*
 *  Created on: Aug 28, 2016                                                                *
 *      Author: eliaschr                                                                    *
 *******************************************************************************************/

/********************************************************************************************
 * Includes                                                                                 *
 *******************************************************************************************/
#include "GenericFuncs.h"


/********************************************************************************************
 * Definitions to control the whole code behaviour                                          *
 *******************************************************************************************/


/********************************************************************************************
 * Constants definitions                                                                    *
 *******************************************************************************************/


/********************************************************************************************
 * Variable definitions                                                                     *
 *******************************************************************************************/


/********************************************************************************************
 * Function declarations                                                                    *
 *******************************************************************************************/


/********************************************************************************************
 * Generic Helper Functions                                                                 *
 *******************************************************************************************/

//********************************************************************************************
/*Parses a string as a hexadecimal, decimal or binary number. The default is a decimal number.
There are two functions, one for 16 bits target value and one for 32 bits target value. The
full implementation is made in 32 bits (ParseNumber32). The 16 bits variant (ParseNumber) uses
the 32 bits one and truncates the value before passing it to the target variable.
If there is a need for a hexadecimal number then there must be a prefix of '0x' to the hex
number, or a postfix of 'h'. If there are both no harm is made. In the same way a binary
number is specified by a prefix of '0b' or a postfix of 'b'. Using both does not harm the
parsing. The strings can be included in single or double quotes or even no quotes can be used.
The parsing is not case sensitive; it accepts both capital and small letters in prefix, main
number or postfix.
The input parameters are:
InStr points to the string containing the number to be parsed,
OutVal is a pointer to the target variable that receives the value parsed
The function returns the number of characters used, including the terminating ones. If there
was an error, that number is negative
*/
int8_t ParseNumber32(char* InStr, uint32_t* OutVal) {
	#define NUM_BIN		(1<<0)				//Specifies if a possible binary number is used
	#define NUM_DEC		(1<<1)				//Specifies if a possible decimal number is used
	#define NUM_HEX		(1<<2)				//Specifies if a possible hex number is used
	#define NUM_PREFIX	(1<<3)				//Found 0x prefix for hex number notation
	#define NUM_POSTFIX	(1<<4)				//Found h postfix for hex number notation
	#define NUM_MAYPRE	(1<<5)				//Found 0b prefix. It can be a binary number
											// specifier or part of a hex number
	#define NUM_MAYPOST	(1<<6)				//Found b postfix. It can be a binary number
  											// specifier or part of a hex number

	int Skipped = 0;						//Counts the number of characters used
	uint32_t Bin = 0,						//Will contain the value in dec if the input
	Dec = 0,								// value is in binary, decimal or hexadecimal
	Hex = 0;								// respectively
	char Quote = *InStr;					//Get the quote character
	char NumFlag = NUM_DEC | NUM_HEX | NUM_BIN;
	char TempChr = 0;						//Current character manipuated

	//Skip the first space characters (if they exist)
	while(Quote == ' ') {					//Need to skip space characters (if they exist)
		Skipped++;							//Another one character skipped
		Quote = *(++InStr);					//Fetch next character
	}
	//Find the quote character and its kind (single or double), if it is used
	if((Quote != '"') && (Quote != '\'')) {	//Is it a kind of quote? (single or double)
		Quote = '\0';						//No => Quote is zeroed
	} else {								//Yes =>, Keep Quote kind
		InStr++;							//Skip this character, also
		Skipped++;							//Another one character skipped
	}
	//Find the prefix that defines the type of number used, if it is used, and if not, skip
	// the leading 0s, if they exist
	if(*InStr == '0') {						//Is first 0?
		InStr++;							//Must check the following character to see if it
											// is a prefix character
		Skipped++;							//Add another character in the skipped ones
		TempChr = *InStr & 0xDF;			//Get the character and convert it to capital
		if(TempChr == 'X') {				//Hex prefix?
			NumFlag = NUM_HEX | NUM_PREFIX;	//Then it is a hex number with prefix (no postfix
											// allowed)
			InStr++;						//Go to next character
			Skipped++;						//Add another character in the list of used
		} else if(TempChr == 'B') {			//Bin possible prefix? (may be hex digit)
			NumFlag = NUM_HEX | NUM_BIN | NUM_MAYPRE;//Perhaps hex, perhaps bin with prefix
			InStr++;						//Advance to next character
			Skipped++;						//Add another one to those used
			Hex = 11;						//If it is hex we must include it in accounting
		} else {							//else, we need to skip possible prepended 0s
			while(*InStr == '0') {			//Skip the first zero digits if they exist
				InStr++;					//Advance pointer
				Skipped++;					//Another skipped character
			}
		}
	}

	//Now lets parse the rest of the number (main one)
	while((NumFlag & (NUM_BIN | NUM_DEC | NUM_HEX)) != 0) {
		TempChr = *InStr;					//Get current character
		if((TempChr == 'h') || (TempChr == 'H')) {//Postfix H found
			NumFlag = NUM_HEX | NUM_POSTFIX;//Flag that we found a postfix specifier for hex
			TempChr = *(++InStr);			//Get the new character
			Skipped++;						//Another character used
		} else if(((TempChr == 'b') || (TempChr == 'B')) &&
			((NumFlag & (NUM_PREFIX | NUM_BIN)) == NUM_BIN)) {
			NumFlag |= NUM_MAYPOST;			//Flag that it may be a postfix for binary
			NumFlag &= ~NUM_DEC;			//Cannot be a decimal number
			Skipped++;						//Add another character used
			TempChr = *(++InStr);			//Get the following character
			Hex = Hex *16 + 11;				//In case of a Hex number, count this in
		}
		if(((TempChr == Quote) && (Quote != '\0')) ||
			(((TempChr <= ' ') || (TempChr == ',') || (TempChr == '.') || (TempChr == ':'))
				&& (Quote == '\0'))) {
			if((NumFlag & NUM_MAYPOST) != 0) {
				NumFlag &= ~(NUM_HEX);		//The final number is not hexadecimal
			}
			Skipped++;						//Also count in this terminating character
			break;							//Normal exit, OK
		} else if((NumFlag & NUM_POSTFIX) != 0){//Do we have a postfix? => Error
			Skipped++;						//Yes => Add this character also
			NumFlag &= ~(NUM_HEX | NUM_DEC | NUM_BIN);//No type of number found
			break;							//Exit with error
		}
		NumFlag &= ~NUM_MAYPOST;			//So, no postfix found
		TempChr -= '0';						//Convert digit to value
		if(TempChr > 1) {					//Not valid binary character?
			NumFlag &= ~NUM_BIN;			//Exclude binary
		}
		if(TempChr > 9) {					//Not a valid decimal character?
			NumFlag &= ~NUM_DEC;			//Exclude decimal
			TempChr -= 7;					//Convert Capital Hex digit to number
			if(TempChr > 15) {
				TempChr -= 32;				//Convert lower case hex digit to number
			}
			if((TempChr < 10) || (TempChr > 15)) {//A symbol between 9 and A or above F is
											// invalid
				NumFlag &= ~NUM_HEX;		//Also clear the hex flag
				break;						//Exit the loop. It is invalid digit
			}
		}
		if((NumFlag & NUM_HEX) != 0) {		//Possible hex number?
			Hex = Hex *16 + TempChr;		//Include current digit
		}
		if((NumFlag & NUM_DEC) != 0) {		//Possible decimal number?
			Dec = Dec *10 + TempChr;		//Include current digit
		}
		if((NumFlag & NUM_BIN) != 0) {		//Possible binary number?
			Bin = Bin *2 + TempChr;			//Include current digit
		}
		InStr++;							//Go to next character to be parsed
		Skipped++;							//Count in another one character
	}
	//In case of error, no type is help, so exit
	if((NumFlag & (NUM_HEX | NUM_DEC | NUM_BIN)) == 0) {
		return -Skipped;
	}
	//Now we have to decide which numbering system to use
	if((NumFlag & NUM_HEX) != 0) {			//Used hexadecimal? Yes =>
		if((NumFlag & (NUM_POSTFIX | NUM_PREFIX)) != 0) {//Valid if only there was a
											// post/prefix
			*OutVal = Hex;					//Output value comes from hexadecimal number
			return Skipped;
		}
	}
	if((NumFlag & NUM_BIN) != 0) {
		if((NumFlag & (NUM_MAYPRE | NUM_MAYPOST)) != 0) {
			*OutVal = Bin;					//Output value comes from a binary number
			return Skipped;
		}
	}
	if((NumFlag & NUM_DEC) != 0) {
		if((NumFlag & (NUM_MAYPRE | NUM_PREFIX | NUM_MAYPOST | NUM_POSTFIX)) == 0) {
			*OutVal = Dec;					//Output value comes from a decomal number
			return Skipped;
		}
	}
	return -Skipped;						//In case of error the returned number is
											// negative
}

/*The 16 bits variant of this function follows.*/
int8_t ParseNumber(char* InStr, uint16_t* OutVal) {
	int8_t Skipped;							//Counts the number of characters used
	uint32_t Result;						//Will get the resulting number

	Skipped = ParseNumber32(InStr, &Result);//Parse the number
	*OutVal = Result & 0xFFFF;				//Truncate the lower 16 bits and store them to the
											// target variable
	return Skipped;							//Return the number of characters used
}


//********************************************************************************************
/* SkipCRLF
Counts the number of CR or LF characters needed to be skipped in order to reach the
beginning of a new line in the input buffer.
Input parameters:
 	InBuffer is a pointer to the buffer that will be scanned
 	InSize is the size of the InBuffer

Returns:
 	- In normal operation it returns the number of CR and LF characters found in a raw
 	- If there was an invalid character (not litteral and not CR or LF) returns -1
 	- If all the characters in InBuffer are CR or LF then it returns 0 as it could never find
 		a normal character = a new line starting
*/
int16_t SkipCRLF(char* InBuffer, int16_t InSize) {
#define FOUNDFLG	(1<<1)
#define FAILFLG		(1<<0)
	int16_t count;
	uint8_t flags;

	count = 0;
	flags = 0;
	while(InSize > 0) {
		if((*InBuffer == '\r') || (*InBuffer == '\n')) {
			flags = FOUNDFLG;
			count++;
			InBuffer++;
			InSize--;
			continue;
		}
		if((*InBuffer < ' ') || (*InBuffer > 0x7F)) {
			flags = FAILFLG;
			count = -1;
			break;
		}
		flags = FOUNDFLG;
		break;
	}
	if(flags == 0) {
		count = 0;
	}
	return count;
}


//********************************************************************************************
/* FindCRLF
Counts the number of characters needed to be skipped in order to find the first CR or LF
character

Input parameters:
 	InBuffer is a pointer to the buffer that will be scanned
 	InSize is the size of the InBuffer

Returns:
 	- In normal operation it returns the number of characters found in a raw
 	- If there was an invalid character (not litteral and not CR or LF) returns -1
 	- If all the characters in InBuffer are CR or LF then it returns 0 as it could never find
 		a normal character = a new line starting
*/
int16_t FindCRLF(char* InBuffer, int16_t InSize) {
	int16_t count;
	uint8_t flags;

	count = 0;
	flags = 0;
	while(InSize > 0) {
		if((*InBuffer == '\r') || (*InBuffer == '\n')) {
			flags = FOUNDFLG;
			break;
		}
		if((*InBuffer < ' ') || (*InBuffer > 0x7F)) {
			flags = FAILFLG;
			count = -1;
			break;
		}
		count++;
		InBuffer++;
		InSize--;
	}
	if(flags == 0) {
		count = 0;
	}
	return count;
}


//********************************************************************************************
/*Gets a two digit ASCII value that represents a hex byte and converts it to integer. If there
is an invalid characted, then it returns -1. The input of the function is a pointer to the two
digit hex ASCII (InVal) and the length of the hex to be interpreted, in digits (Len)*/
int16_t Hex2Int(char *InVal, uint8_t Len) {
  int16_t OutVal, DigVal;
  uint8_t i;

  OutVal = 0;								//Clear the resulting value
  for(i = 0; i < Len; i++) {				//Repeat for as many digits as Len defines
	DigVal = InVal[i] - '0';				//Get current digit value
	if(DigVal >= 10) {						//Greater than 10? => Need to
	  DigVal -= 7;							//subtract other 7 becase of '9' to 'A' ASCII gap
	}
	if((DigVal >= 16) || (DigVal <0)) {		//Value out of bounds?
	  OutVal = -1;							//Well... Flag the value as -1
	  break;								//and stop interpretting more digits
	}
	OutVal = OutVal *16 + DigVal;			//Digit OK => Include it in final calculation
  }
  return OutVal;							//Return the converted value
}


//********************************************************************************************
/*Converts an integer number to ASCII string in a target buffer. The size of the buffer is
specified by BufLen in order not to have buffer overflow situations. The return value is the
number of digits converted. No terminating character ('\0') is inserted in the target buffer.
The converted string appears at the last cells of the target buffer.
Tip on usage: Since we have an integer (16 bits), a buffer of 6 characters (indexed 0 to 5) is
enough. By calling this function with BufLen equal to real_buffer_length -1, in our example 5,
and placing the terminating character in the last position of the buffer (in our example
Buffer[5] = '\0'), then upon return of this function the string that can be used starts at
position (real_buffer_length* -1 -return_value) = (BufLen -return_value). Lets see a coding
example:
	int TempInt = 1352;						//Value to be converted
	char* Buffer[6];						//Storage buffer of target string
	Buffer[5] = '\0';						//Last Buffer cell is the terminating character
	int i = Int2Ascii(TempInt, Buffer, 5);	//Convert TempInt to string in Buffer. Returns 4
	print(&Buffer[5 -i]);					//prints the string of 4 characters + 1 the
											// terminating one, from position 1 of Buffer and
											// not position 0!
*/
int16_t Int2Ascii(int16_t InVal, char* Buffer, int16_t BufLen) {
  int16_t j = BufLen;						//Starting position in target buffer is its final
  											// cell
  do {
	j--;									//One more digit in buffer
	Buffer[j] = (InVal %10) + '0';			//Store the last digit (Less Significant one)
	InVal /= 10;							//Divide the value by 10 to discard the last digit
  } while((InVal > 0) && (j >= 0));			//Repeat as long as the value contains more digits
  											// and the buffer contains more empty cells
  return (BufLen -j);						//Return the number of characters used in target
  											// buffer
}


//********************************************************************************************
/*Finds the string size, in characters. The string is composed of normal ASCII characters from
space character to 0x7F. All other characters are considered as string termination. The return
value is the pure number of characters in the string, without the terminating one.
*/
uint16_t StrSize(char* InStr) {
	uint16_t counter;

	counter = 0;
	while((InStr[counter] >= ' ') && (InStr[counter] <= 0x7F)) {
		counter++;
	}
	return counter;
}


//********************************************************************************************
/*Calculates CRC16 of data. The polynomial used is (x^16 +x^15 +x^2 +1). If another polynomial
is needed, it can be changed through the CRC16_POLY definition. When the checksum calculation
is started, the input parameter OldCRC should be DEF_CRC16 which is defined as all bits set
(0xFFFF).
Input:
	InByte: is the byte to be included in the CRC16 checksum
	OldCRC: is the old CRC16 value calculated for the previous bytes. If InByte is the first
		one in the series of bytes that needed to be included in CRC16, then OlCRC should be
		equal to DEF_CRC16
Returns: the new CRC16 value
Example of usage:
uint16_t CRC16Value;
uint8_t i;

CRC16Value = DEF_CRC16;
for(i = 0; i < sizeof(ArrayOfBytesForCRC); i++) {
	CRC16Value = CalcCRC16( ArrayOfBytesForCRC[i], CRC16Value);
}
//Now CRC16Value contains the CRC16 checksum of all bytes in array ArrayOfBytesForCRC
*/
uint16_t CalcCRC16(uint8_t InByte, uint16_t OldCRC) {
	uint8_t i;								//Counter for bits of InByte

	for(i = 0; i < 8; i++) {
		if(((OldCRC & 0x8000) >> 8) ^ (OldCRC & 0x80)) {
			OldCRC = (OldCRC << 1) ^ CRC16_POLY;
		} else {
			OldCRC = OldCRC << 1;
		}
		OldCRC <<= 1;
	}
	return OldCRC;
}


//********************************************************************************************
/*Converts the byte value of the input to a string of two hex characters that represent the
input value in Hex format. The input of the function is the byte value to be converted (InVal)
and a pointer to the two digit hex ASCII buffer that will hold the resulting string. No
termination '\0' is appended, thus the resulting string is always two characters long.*/
void Byte2Hex(uint8_t InVal, char *OutStr) {
	uint8_t tempVal;

	tempVal = (InVal >> 4);
	if(tempVal > 9) {
		tempVal += 7;
	}
	OutStr[0] = tempVal +0x30;
	tempVal = InVal & 0x0F;
	if(tempVal > 9) {
		tempVal += 7;
	}
	OutStr[1] = tempVal +0x30;
}
