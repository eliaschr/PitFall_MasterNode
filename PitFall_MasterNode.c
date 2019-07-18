/*********************************************************************************************
* PitFall_MasterNode.c                                                                       *
**********************************************************************************************
*  Created on: Sep 26, 2016                                                                  *
*      Author: eliaschr                                                                      *
**********************************************************************************************
* The communications set is formed by many Leaf Nodes that are connected to a Master Node.   *
* The Master Node is the gateway of the Leaf nodes to the Internet. Every node registered to *
* the Master one can communicate data to it and the Master Node sends those data to the      *
* internet server through GPRS communication.                                                *
*                                                                                            *
*============================================================================================*
* Master Node explanation                                                                    *
*--------------------------------------------------------------------------------------------*
* The master node's code takes care of data that come from leaf nodes and, when ready, it    *
* sends them to the internet server. The main processor is connected to an RF circuit to be  *
* able to communicate with the leaf nodes and to a GSM module to use for GPRS communication. *
*                                                                                            *
* The program is based on Texas Instrument's real time operating system, TI-RTOS and its     *
* EasyLink framework, enriched to include collision detection.                               *
*/
/* XDCtools Header files */
#include <MainTask.h>
#include <RF/RadioTask.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>

/* TI-RTOS Drivers Header files */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"

#include "PitFall_MasterNode.h"
#include "Flash/AppFlash.h"
#include "GSM/UART_GPRS.h"

/*Pin driver handles.*/
PIN_Handle ledPinHandle;
PIN_State ledPinState;
PIN_Handle swPinHandle;
PIN_State swPinState;

CallbackFunc KeyUpCb;
CallbackFunc KeyDownCb;
CallbackFunc KeyLeftCb;
CallbackFunc KeyRightCb;
CallbackFunc KeySelectCb;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
const PIN_Config ledPinTable[] = {
	BoardLedR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	BoardLedY | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	BoardLedG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	BoardLedB | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE
};

const PIN_Config swPinTable[] = {
	BUTTON_L	| PIN_INPUT_EN		| PIN_PULLUP	| PIN_IRQ_NEGEDGE	| PIN_BM_HYSTERESIS,
	BUTTON_R	| PIN_INPUT_EN		| PIN_PULLUP	| PIN_IRQ_NEGEDGE	| PIN_BM_HYSTERESIS,
	BUTTON_U	| PIN_INPUT_EN		| PIN_PULLUP	| PIN_IRQ_NEGEDGE	| PIN_BM_HYSTERESIS,
	BUTTON_D	| PIN_INPUT_EN		| PIN_PULLUP	| PIN_IRQ_NEGEDGE	| PIN_BM_HYSTERESIS,
	BUTTON_SEL	| PIN_INPUT_EN		| PIN_PULLUP	| PIN_IRQ_NEGEDGE	| PIN_BM_HYSTERESIS,
	PIN_TERMINATE
};


/* Function declarations */
void ButtonCbFxn(PIN_Handle handle, PIN_Id pinId);


//********************************************************************************************
/*Lights up the red led to inform for error condition and enters an infinite loop disabling
interrupts. The input variable points to an explanation string for the error condition. The
function never returns.*/
void Halt_abort(char* ErrorTxt, void* ptr) {
	if(ledPinHandle) {
		PIN_setOutputValue(ledPinHandle, BoardLedR, 1);
	}
	Hwi_disable();
	while(1);
}

//********************************************************************************************
/*Callback function for key presses. This function is a dispatcher, so every other task can
use the buttons.*/
void ButtonCbFxn(PIN_Handle handle, PIN_Id pinId) {
	if((pinId == BUTTON_L) && (KeyLeftCb != NULL)) {
		KeyLeftCb();
	}
	if((pinId == BUTTON_R) && (KeyRightCb != NULL)) {
		KeyRightCb();
	}
	if((pinId == BUTTON_U) && (KeyUpCb != NULL)) {
		KeyUpCb();
	}
	if((pinId == BUTTON_D) && (KeyDownCb != NULL)) {
		KeyDownCb();
	}
	if((pinId == BUTTON_SEL) && (KeySelectCb != NULL)) {
		KeySelectCb();
	}

}


/*********************************************************************************************
Sets callback functions for a key button
Input:
	pinId: is the ID of the button to be used. Can be one of BUTTON_L, BUTTON_R, BUTTON_U,
		BUTTON_D or BUTTON_SEL
	cbFxn: is the callback function to be used. It is of type CallbackFunc
*/
void registerButtonCb(PIN_Id pinId, CallbackFunc cbFxn) {
	switch(pinId) {
	case BUTTON_L:
		KeyLeftCb = cbFxn;
		break;

	case BUTTON_R:
		KeyRightCb = cbFxn;
		break;

	case BUTTON_U:
		KeyUpCb = cbFxn;
		break;

	case BUTTON_D:
		KeyDownCb = cbFxn;
		break;

	case BUTTON_SEL:
		KeySelectCb = cbFxn;
		break;
	}
}


/*
 *  ======== main ========
 */
int main(void) {
	/*Board initialization.*/
	Board_initGeneral();					//Initialize board pins to default status
	Board_initUART();						//Init UART pins
	InitFlash();							//Initialize the GPRS variables in Flash memory

	/*GPRS and RF tasks use leds and switches. It is a good idea to initialize these handles
	before using them.*/
	/*Open LED pins.*/
	ledPinHandle = PIN_open(&ledPinState, ledPinTable);
	if(!ledPinHandle) {
		Halt_abort("Error initializing board LED pins\n", &ledPinState);
	}

	/*Open Switch pins and reset switches callback functions. Finally, register the callback
	function for interrupts generated by switches.*/
	swPinHandle = PIN_open(&swPinState, swPinTable);
	if(!swPinHandle) {
		Halt_abort("Error initializing board switch pins\n", &swPinState);
	}
	KeyUpCb = NULL;
	KeyDownCb = NULL;
	KeyLeftCb = NULL;
	KeyRightCb = NULL;
	KeySelectCb = NULL;
	PIN_registerIntCb(swPinHandle, ButtonCbFxn);

	/*Initialize other tasks of the application.*/
	InitGPRSTask();							//Prepare the GSM/GPRS task
	InitRFTask();							//Prepare the RF task
	MainTask_init();						//Prepare the main synchronization task

	BIOS_start();							//Now system fires the task functions

	/*Code should never reach this line...*/
	return (0);
}
