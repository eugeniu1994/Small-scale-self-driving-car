//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/         >>> MOTOR CONTROLLER
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: 		Wild_Thumper_Motor.c
// Version: 	1.0
// Target: 		Wild Thumper Main Controller - ATMEGA664 @20.00 MHz
// Author(s): 	Bastiaan Scherphof 
// 			  	Hein Wielink 
//////////////////////////////////////////////////////////////////////////////////////
// Description:
// Control of the Wild Thumper
// This program handles:
// - I2C as master 
// - UART 0 
// - UART 1
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	includes																		//
//																					//
//////////////////////////////////////////////////////////////////////////////////////
#include <avr/io.h>         		// General I/O Declarations
#include <avr/pgmspace.h>   		// tabel in flash
#include <avr/interrupt.h>  		// ext,timer int sei(), cli()
#include <inttypes.h>
#include <avr/eeprom.h>
#include <compat/deprecated.h>  	// stay compatible outp, sbi, cbi ed.
#include <stdbool.h> 
#include <stdio.h>
#include <string.h>
#include <compat/twi.h>
#include "WildThumperLib_Motor.h"	//library
#include "WT_UART.h"				//Include usart functions 


//main loop
int main (void)
{
	//////////////////////////////
	//   Configuration         //
	/////////////////////////////


	portInit();									// Configuration of Inputs and Outputs
		
	USART0_Init(9600);							// Init USART0 

	Timer1_Init();								// Init Timer1

	sei();										// Enable interrupt	

	/////////////////////////////
	//   Start_Up functions    //
	/////////////////////////////

	Timer1_Start();								//Start Timer 1 (Enable Stopwatches and delay functions)

	StartUp_Ledblinking();						//By start-up led blinking 

	startStopwatch1();

//////////////////////////////////////////////////////////////////////////////////////
// this function is being used to write a normal string
//////////////////////////////////////////////////////////////////////////////////////

	USART0_WriteString("Gestart Wild Thumper");	//Write to Uart

//////////////////////////////////////////////////////////////////////////////////////
// this function writes a integer
//////////////////////////////////////////////////////////////////////////////////////

	USART0_WriteInt(1024,DEC);  		// Decimal
 	USART0_WriteInt(044,OCT);			// Ocal
 	USART0_WriteInt(0b11010111,BIN);	// Binary

//////////////////////////////////////////////////////////////////////////////////////
// this function writes a integer with a specified length
//////////////////////////////////////////////////////////////////////////////////////

	USART0_WriteIntLength(1024,DEC,6);  		// Decimal
 	USART0_WriteIntLength(044,OCT,4);			// Ocal
	USART0_WriteIntLength(0b11010111,BIN,8); 	// Binary

//////////////////////////////////////////////////////////////////////////////////////
// this function Writes a string with specified length and offset
//////////////////////////////////////////////////////////////////////////////////////

 	writeStringLength("WildThumper robot\n",17,0);
	// would output: "RP6 Robot Sytem\n"
	writeStringLength("WildThumper robot\n",11,4);
	// would output: "Robot System"
	writeStringLength("WildThumper robot\n",40,4);
	// would output: "Robot System\n"
	// No matter if the specified length is 40 characters!



	/////////////////////////////
	//   Main Loop             //
	/////////////////////////////
	
	
	while(1==1)			
	{
	}
  }


