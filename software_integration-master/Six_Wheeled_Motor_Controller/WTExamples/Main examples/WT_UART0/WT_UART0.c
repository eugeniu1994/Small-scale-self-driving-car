//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/         >>> MAIN CONTROLLER
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: 		WT_UART0.c
// Version: 	1.0
// Target: 		Wild Thumper Main Controller - ATMEGA664 @20.00 MHz
// Author(s): 	Arexx enginering
// 			  	
//////////////////////////////////////////////////////////////////////////////////////
// Description:
// Control of the Wild Thumper
// This program handles:
// - UART0 test
// - 
//////////////////////////////////////////////////////////////////////////////////////



/*****************************************************************************************
**
**
**
**
**	Make sure that you remove the jumper after programming else the uart wouldn't work!
**
**
**
**
*****************************************************************************************/



//Include library
#include "WildThumperLib_Main.h"		//Include Wild Thumper functions

/*****************************************************************************/
// Main function - The program starts here:

int main(void)
{
	portInit(); 		// Always call this first! The Processor will not work
						// correctly otherwise.

	USART0_Init(9600);	//Init USART0

	Timer1_Init();		//Init Timer1
	
	/////////////////////////////
	//   Start_Up functions    //
	/////////////////////////////
	
	Timer1_Start();		//Start Timer 1 (Enable Stopwatches and delay functions)

	sei();				// Start interrupt check



	// Write a text message to the UART:
	USART0_WriteString("Gestart Wild Thumper");		//Write to Uart
	
	USART0_WriteString("\nJust a simple counter program\n\n");
	
	// Define a counting variable:
	uint16_t counter = 0;
	
	// ---------------------------------------
	// Main loop:
	while(true)
	{
			// Now we check what value the counter has, ...
		if(counter < 100) // ... is it smaller than 100?
		{
			// Yes --> output the Counter value with the "writeInteger"
			// function:
			USART0_WriteString("Counter: ");
			USART0_WriteInt(counter, BIN);
			USART0_WriteString("(BIN) | ");
			USART0_WriteInt(counter, OCT);
			USART0_WriteString("(OCT) | ");
			USART0_WriteInt(counter, DEC);
			USART0_WriteString("(DEC) | ");
			USART0_WriteInt(counter, HEX);
			USART0_WriteString("(HEX) \n");
		}
		else 			  // ... or is it greater than or equal to 100?
		{
			// No, the counter >= 100 --> use "writeIntegerLength" instead.
			USART0_WriteString("Counter L: ");
			USART0_WriteIntLength(counter, BIN, 16);  
			USART0_WriteString("(BIN) | ");
			USART0_WriteIntLength(counter, OCT, 6);
			USART0_WriteString("(OCT) | ");
			USART0_WriteIntLength(counter, DEC, 6);
			USART0_WriteString("(DEC) | ");
			USART0_WriteIntLength(counter, HEX, 4);
			USART0_WriteString("(HEX) \n");
		}
		
		counter++;    // Increment counter
		
		mSleep(100); // delay 100ms
	}
	// End of main loop!
	// ---------------------------------------

	return 0;
}
