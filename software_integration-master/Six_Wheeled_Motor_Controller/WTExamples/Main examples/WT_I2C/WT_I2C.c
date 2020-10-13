//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/         >>> MAIN CONTROLLER
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: 		WT_I2C.c
// Version: 	1.0
// Target: 		Wild Thumper Main Controller - ATMEGA664 @20.00 MHz
// Author(s): 	Bastiaan Scherphof 
// 			  	Hein Wielink 
//////////////////////////////////////////////////////////////////////////////////////
// Description:
// Control of the Wild Thumper
// This program handles:
// - I2C test
// - 
//////////////////////////////////////////////////////////////////////////////////////

//Include library
#include "WildThumperLib_Main.h"		//Include Wild Thumper functions
 
//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Variables																		//
//																					//
//////////////////////////////////////////////////////////////////////////////////////


uint8_t block = false;				//Check if I2C line is free
uint8_t result[40];					//Received data from motor controller
uint8_t messageBuf[20]; 			// Buffer for I2C Data
uint8_t NumberOfI2Cerrors = 0;


//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	I2C	functions																	//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

//  I2C interrupt Handler 
void I2C_Event_Handler(void)
  {
    // this code will be called anytime that PCINT31 switches 
    //    (hi to lo, or lo to hi)

	if(!block && (PIND & (1<< PIND7))) 
	{
		block = true; // Block further requests and wait until 
					  // this request has been processed.
		I2CTWI_requestRegisterFromDevice(I2C_WT_ADR, INT0_STATUS_CHECK, 0, 20);

	}

  }

// I2C interrupt Handler 
// This Event Handler is very nice for reacting on an interrupt request 
// from the Slave controller and read all the data from it! 
void I2C_requestedDataReady(uint8_t dataRequestID)
  {
	if(dataRequestID == INT0_STATUS_CHECK) 
	  {                                      
	  	// get received data (6 bytes)
        I2CTWI_getReceivedData(result, 20); 
		
		
	   	//Action: 
		//................
		// statusLEDs.byte = messageBuf[0];
		// updateStatusLEDs();

		// ------------------------------------
		// IMPORTANT - reset the block flag:

		block = false;
	  }
  }

//  I2C error Handler
void I2C_transmissionError(uint8_t errorState)
  {
    NumberOfI2Cerrors++;
	//USART0_WriteString("\nI2C ERROR - TWI STATE: 0x");
	//USART0_Write(errorState);
	block = false;
  }

////////////////////////////////////////////////////////////////////////////////////////
// this function create a looplight. First the direction is beeing set, 
// then the active led is shifted through the siftregister.
// the I2C function I2CTWI_transmit3Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_SET_LEDS, led)
// is used to send: 
// - the adres (I2C_WT_ADR) of the motor controller
// - a write command (WRITE_COMMAND)
// - the command that is going to be send to the motor controller (CMD_SET_LEDS) [see also WildThumperLib_Main.h for the commands]
// - the data that is needed for this command
////////////////////////////////////////////////////////////////////////////////////////

int task_LedI2C(void)
  {
  
	uint8_t dir=1;
	uint8_t led=16;
	uint8_t i=0;
	for (i=0;i<7;i++)
	  {
	  	if (led == 128)
		  {
		  	dir = 0;
		  }
		else if (led == 16)
		  {
		  	dir = 1;
		  }

		if (dir==0)
		  {
			led >>= 1;
		  }
		else if (dir==1)
		  {
			led <<= 1;
		  }
		  I2CTWI_transmit3Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_SET_LEDS, led);
		  mSleep(200);
	  }
	  return 0;
  }

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

	I2CTWI_initMaster(200);				//Init I2C as master (100kHz clock)
	
	/////////////////////////////
	//   Start_Up functions    //
	/////////////////////////////
	
	Timer1_Start();		//Start Timer 1 (Enable Stopwatches and delay functions)

	sei();				// Start interrupt check

	I2CTWI_setTransmissionErrorHandler(I2C_transmissionError);

	I2CTWI_setRequestedDataReadyHandler(I2C_requestedDataReady);

	I2C_setInterruptEventHandler(I2C_Event_Handler);

	startStopwatch4();
	startStopwatch5();


	// ---------------------------------------
	// LEDs:

	LedOnOff(0b00111111); // Turn all LEDs on!
	mSleep(1000); // delay 1000ms = 1s
	LedOnOff(0b00000000); // All LEDs off!
	mSleep(500); // delay 500ms = 0.5s

	// ---------------------------------------
	// Main loop - the program will loop here forever!
	// In this program, it only runs a small LED chaselight.

	/////////////////////////////
	//   Main Loop             //
	/////////////////////////////
 //return 0;
	while(1==1)			
	  {
		task_LedI2C();
	  }
	 
  }


