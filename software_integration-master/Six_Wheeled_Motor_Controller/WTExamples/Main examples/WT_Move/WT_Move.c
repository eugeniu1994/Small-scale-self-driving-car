//////////////////////////////////////////////////////////////////////////////////////
//                           _______________________
//                      \| WILD THUMPER  ROBOT SYSTEM |/
//                            \_-_-_-_-_-_-_-_-_-_/         >>> MAIN CONTROLLER
// ----------------------------------------------------------------------------
// ------------------- [c]2010 / 2011 - AREXX ENGINEERING ---------------------
// -------------------------- http://www.arexx.com/ ---------------------------
//////////////////////////////////////////////////////////////////////////////////////
// File: 		Wild_Thumper_Main.c
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
// - APC Dongle communication
// -
//////////////////////////////////////////////////////////////////////////////////////

//Include library
#include "WildThumperLib_Main.h"		//Include Wild Thumper functions
#include "WT_I2Cmaster.h"				//Include I2C functions

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Defines																			//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

#define I2C_WT_ADR 					10				// The default address of the Master Controller
#define INT0_STATUS_CHECK 			0
#define WRITE_COMMAND				0				//

//I2C Registers Read
#define I2C_REG_STATUS	 		 	1		//Status of the motor controller

#define I2C_REG_SPEED_MOTOR_1		2		//Actual speed of motor 1
#define I2C_REG_SPEED_MOTOR_2	 	3		//Actual speed of motor 2
#define I2C_REG_SPEED_MOTOR_3 	 	4		//Actual speed of motor 3
#define I2C_REG_SPEED_MOTOR_4	 	5		//Actual speed of motor 4
#define I2C_REG_SPEED_MOTOR_5	 	6		//Actual speed of motor 5
#define I2C_REG_SPEED_MOTOR_6 	 	7		//Actual speed of motor 6
#define I2C_REG_SPEED_ALL			8		//Actual speed of all motors

#define I2C_REG_CURR_MOTOR_1		10		//Actual current of motor 1
#define I2C_REG_CURR_MOTOR_2 	 	11		//Actual current of motor 2
#define I2C_REG_CURR_MOTOR_3 	 	12		//Actual current of motor 3
#define I2C_REG_CURR_MOTOR_4	 	13		//Actual current of motor 4
#define I2C_REG_CURR_MOTOR_5 	 	14		//Actual current of motor 5
#define I2C_REG_CURR_MOTOR_6 	 	15		//Actual current of motor 6

#define I2C_REG_ENCODER_ERRORS	 	16		//Status from the encoders
#define I2C_REG_MOTOR_ERRORS	 	17		//All error flags from the motors
#define I2C_REG_CURRENT_ERRORS	 	18		//All error flags from the motors

#define I2C_REG_LEDS	 		 	22		//Actual status of the four leds
#define I2C_TEST_I2C				23		//This register is used for test the I2C communication


//I2C Registers Write (commands)

#define CMD_STOP_ALL				1		//Command Wild Thumper: STOP WILD THUMPER
#define CMD_MOTORS_FORWARD_LEFT		2		//Command Wild Thumper: MOVE FORWARD / LEFT
#define CMD_MOTORS_FORWARD_RIGHT	3		//Command Wild Thumper: MOVE FORWARD / RIGHT
#define CMD_MOTORS_BACKWARD_LEFT	4		//Command Wild Thumper: MOVE BACKWARD / LEFT
#define CMD_MOTORS_BACKWARD_RIGHT	5		//Command Wild Thumper: MOVE BACKWARD / RIGHT

#define CMD_CHANGE_MOTOR_1			10		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 1
#define CMD_CHANGE_MOTOR_2			11		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 2
#define CMD_CHANGE_MOTOR_3			12		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 3
#define CMD_CHANGE_MOTOR_4			13		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 4
#define CMD_CHANGE_MOTOR_5			14		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 5
#define CMD_CHANGE_MOTOR_6			15		//Command Wild Thumper: CHANGE SPEED AND DIRECTION MOTOR 6
#define CMD_SPEED_MOTOR_1			16		//Command Wild Thumper: CHANGE SPEED MOTOR 1
#define CMD_SPEED_MOTOR_2			17		//Command Wild Thumper: CHANGE SPEED MOTOR 2
#define CMD_SPEED_MOTOR_3			18		//Command Wild Thumper: CHANGE SPEED MOTOR 3
#define CMD_SPEED_MOTOR_4			19		//Command Wild Thumper: CHANGE SPEED MOTOR 4
#define CMD_SPEED_MOTOR_5			20		//Command Wild Thumper: CHANGE SPEED MOTOR 5
#define CMD_SPEED_MOTOR_6			21		//Command Wild Thumper: CHANGE SPEED MOTOR 6
#define CMD_DIR_MOTOR_1				22		//Command Wild Thumper: CHANGE DIRECTION MOTOR 1
#define CMD_DIR_MOTOR_2				23		//Command Wild Thumper: CHANGE DIRECTION MOTOR 2
#define CMD_DIR_MOTOR_3				24		//Command Wild Thumper: CHANGE DIRECTION MOTOR 3
#define CMD_DIR_MOTOR_4				25		//Command Wild Thumper: CHANGE DIRECTION MOTOR 4
#define CMD_DIR_MOTOR_5				26		//Command Wild Thumper: CHANGE DIRECTION MOTOR 5
#define CMD_DIR_MOTOR_6				27		//Command Wild Thumper: CHANGE DIRECTION MOTOR 6
#define CMD_SET_LEDS 				28		//Command Wild Thumper: CHANGE LEDS
#define CMD_TEST_I2C				30		//This register is used for test the I2C communication

#define CMD_PID_P					33
#define CMD_PID_I					34
#define CMD_PID_D					35

//////////////////////////////////////////////////////////////////////////////////////
//																					//
//	Variables																		//
//																					//
//////////////////////////////////////////////////////////////////////////////////////

uint8_t block = false;				//Check if I2C line is free
uint8_t result[40];					//Received data from motor controller
uint8_t messageBuf[20]; 			// Buffer for I2C Data
uint8_t NumberOfI2Cerrors = 0;


void Drive_Control (char Direction, char Speed, char Angle)
  {
  	  Autonomous = false;
   	  switch (Direction)
       	 {
		  case 0: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_STOP_ALL, 			Speed, Angle); break;
	 	  case 1: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_FORWARD_LEFT, Speed, Angle); break;
		  case 2: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_FORWARD_RIGHT, Speed, Angle); break;
		  case 3: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_BACKWARD_LEFT, Speed, Angle); break;
		  case 4: I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_BACKWARD_RIGHT, Speed, Angle); break;
		  case 6: Autonomous = true; break;
		}
  }


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
	USART0_WriteString("\nI2C ERROR - TWI STATE: 0x");
	USART0_Write(errorState);
	block = false;
  }



void task_Sensors (void)
  {
	Sensor.BUMPER_R = get_bumper_right();
	Sensor.BUMPER_L = get_bumper_left();
	Sensor.ACS_R = ACS_Check_Right();
	mSleep(1);
	Sensor.ACS_L = ACS_Check_Left();
	mSleep(1);
	Sensor.ACS_RF = ACS_Check_Front_Right();
	mSleep(1);
	Sensor.ACS_LF = ACS_Check_Front_Left();

  }

void task_Application_Timeout (void)
  {
	if(getStopwatch5() > 1000)
	  	  {
			I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_STOP_ALL, 0, 0);
		  }

  }

void Drive_autonomous(void)
  {

   	if (getStopwatch6() > 2000)
  	  {
		I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_FORWARD_LEFT, 90, 0);
		setStopwatch6(0);
		stopStopwatch6();
		Obstacle_Right = false;
		Obstacle_Left = false;
	  }

	else if (Sensor.BUMPER_R || Sensor.ACS_R || Sensor.ACS_RF || Obstacle_Right)
	  {
	  	startStopwatch6();
 		if (Obstacle_Right )I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_BACKWARD_RIGHT, 126, 6);
		Obstacle_Right = true;
	  }

	 else if (Sensor.BUMPER_L || 	Sensor.ACS_L || Sensor.ACS_LF || Obstacle_Left)
      {
	 	startStopwatch6();
		if (Obstacle_Left)	I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_BACKWARD_LEFT, 126, 6);
		Obstacle_Left = true;
	  }

	 else I2CTWI_transmit4Bytes(I2C_WT_ADR, WRITE_COMMAND, CMD_MOTORS_FORWARD_LEFT, 90, 0);

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

	USART0_Init(9600);					//Init USART0

	USART1_Init(9600);					//Init USART1

	Timer1_Init();						//Init Timer1

	I2CTWI_initMaster(200);				//Init I2C as master (100kHz clock)

	ADC_Init();

	initACS();

	PCMSK3 |= (1 << PCINT31);
  	PCICR |= (1 << PCIE3);
	EICRA = (1<<ISC21)|(0<<ISC20);
	EIMSK = (1<<INT2);

	sei();								// Enable interrupt


	/////////////////////////////
	//   Start_Up functions    //
	/////////////////////////////

	Timer1_Start();									//Start Timer 1 (Enable Stopwatches and delay functions)

	StartUp_Ledblinking();							//By start-up led blinking

	mSleep(500);

	USART0_WriteString("EREN GOKSU LOL");		//Write to Uart

	sei();

	//////////////////////////////////////
	//   Register Event Handlers I2c    //
	//////////////////////////////////////

	I2CTWI_setTransmissionErrorHandler(I2C_transmissionError);

	I2CTWI_setRequestedDataReadyHandler(I2C_requestedDataReady);

	I2C_setInterruptEventHandler(I2C_Event_Handler);

//	RF_SetReceivedDataHandler(RF_ReceivedDataHandler1);

	startStopwatch4();
	startStopwatch5();

		LedOnOff(0b00000001);
		Drive_Control (CMD_MOTORS_FORWARD_LEFT, 50, 0);
		mSleep(500);
		Drive_Control (CMD_MOTORS_FORWARD_RIGHT, 50, 0);
		LedOnOff(0b00010000);
		mSleep(1000);
		Drive_Control (CMD_STOP_ALL, 0, 0);
		mSleep(100);
		LedOnOff(0b00000010);
		Drive_Control (CMD_MOTORS_BACKWARD_LEFT, 50, 0);
		mSleep(500);
		Drive_Control (CMD_MOTORS_BACKWARD_RIGHT, 50, 0);
		LedOnOff(0b00100000);
		mSleep(1000);
		Drive_Control (CMD_STOP_ALL, 0, 0);
		mSleep(100);
		LedOnOff(0b00111111);
		Autonomous = true; 				//WT start drving autonomous



	/////////////////////////////
	//   Main Loop             //
	/////////////////////////////

	while(1==1)
	  {

	  	if(getStopwatch4() > 100 && block == false)
	  	  {
	    	//update of the register from the motor controller
			//This will happend every 200ms
			block = true;
			I2CTWI_transmitByte(10,0);
			I2CTWI_readRegisters(10,0,result,20);
			block = false;
			setStopwatch4(0);
	  	  }

	  	task_I2CTWI();
	  	task_Sensors();
		task_Application_Timeout();

		if (Autonomous == true && (getStopwatch5() < 1000))

	 	  {
		  Drive_autonomous();
	  	  }
	  }
  }
