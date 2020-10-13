//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/         >>> MAIN CONTROLLER
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: 		WT_Led.c
// Version: 	1.0
// Target: 		Wild Thumper Main Controller - ATMEGA664 @20.00 MHz
// Author(s): 	Arexx enginering 
//////////////////////////////////////////////////////////////////////////////////////
// Description:
// Control of the Wild Thumper
// This program handles:
// - Led test
// - 
//////////////////////////////////////////////////////////////////////////////////////

//Include library
#include "WildThumperLib_Main.h"		//Include Wild Thumper functions
 
//////////////////////////////////////////////////////////////////////////////////////
//																					//
//  Main loop																		//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

int main (void)
  {
	//////////////////////////////
	//   Configuration          //
	//////////////////////////////
	
	portInit();							//Set Input and Outputs

	Timer1_Init();						//Init Timer1
	
	/////////////////////////////
	//   Start_Up functions    //
	/////////////////////////////
	
	Timer1_Start();		//Start Timer 1 (Enable Stopwatches and delay functions)

	sei();				// Start interrupt check

	// ---------------------------------------
	// LEDs:

	LedOnOff(0b00111111); // Turn all LEDs on!

	// 0b111111 is a binary value and is the same as
	// 63 in the decimal system.
	// For this routine, the binary value is better to read, because each bit
	// represents one of the LEDs.
	// e.g. this:
	// LedOnOff(0b00000001); would set only LED1
	// LedOnOff(0b00000010); LED2
	// LedOnOff(0b00000100); LED3
	// LedOnOff(0b00101001); LED6, LED4, LED1 - and so on!
	// the first two bits are used for Q7 and Q8
	// see also the WT mainboard schematics

	mSleep(1000); // delay 1000ms = 1s
	LedOnOff(0b00000000); // All LEDs off!
	mSleep(500); // delay 500ms = 0.5s


	// ---------------------------------------

	uint8_t runningLight = 1; // This defines the local unsigned 8 bit variable "runningLight".
						      // It can be accessed everywhere _below_ in this function.
						      // And ONLY within this function!

	// ---------------------------------------
	// Main loop - the program will loop here forever!
	// In this program, it only runs a small LED chaselight.

	/////////////////////////////
	//   Main Loop             //
	/////////////////////////////

	while(1==1)			
	  {
		// Here we do a small LED test:
		// ---------------------------------------
		LedOnOff(runningLight); 	// Set status LEDs to the value of the variable
							// runningLight.
							// In the first loop iteration it has the value 1,
							// and thus the LED8 will be switched on.

		runningLight <<= 1; // shift the bits of "runningLight" one step to the left.
						// As there is only one bit set in this variable,
						// only one LED is on at the same time.
						// This results is a moving light dot like this:
						// 1: 0b00000001
						// 2: 0b00000010
						// 3: 0b00000100
						// 4: 0b00001000
						// 5: 0b00010000
						// 6: 0b00100000 
						//
						// In decimal format that would be the numbers:
						// 1, 2, 4, 8, 16, 32

		// When we have reached a value > 32 (32 means Status LED6 is on), 
		// we need to reset the value of runningLight to 1 to start again
		// from the beginning...
		// Instead of "32" we could also write "0b00100000".
		if(runningLight > 32)
			runningLight = 1; 	// reset runningLight to 1 (LED8) 

		// If we want to see the running Light, we need to
		// add a delay here - otherwise our human eyes would not see
		// the moving light dot: 
		mSleep(100); // delay 100ms = 0.1s 
		
		// ---------------------------------------	  	
	  }
  }


