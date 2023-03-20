/*
 * Display.h
 *
 * Created: 3/20/2023 9:25:01 AM
 *  Author: ponti
 */ 

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

//Global variables for display strings
volatile uint8_t FirstLineStr[21] =  "                    ";
volatile uint8_t SecondLineStr[21] = "                    ";
volatile uint8_t ThirdLineStr[21] =  "                    ";
volatile uint8_t FourthLineStr[21] = "                    ";

char tenthseconds[21] =  "Tenths: x           "; // modify [8]
char seconds[21]      =  "Seconds: xx         "; // modify [10,11]
char minutes[21]      =  "Minutes: xx         "; // modify [10,11]

//Global variables for TWI state and data
uint8_t display_state =0;
uint8_t TWI_data = 0;
uint8_t TWI_slave_address = 0;
uint8_t TWI_char_index = 0;

volatile int lcd_counter = 0;

//Define the codes for TWI actions
#define TWCR_START 0xA4|(1<<TWIE) //send start condition
#define TWCR_STOP 0x94|(1<<TWIE) //send stop condition
#define TWCR_RSTART 0xA4|(1<<TWIE) //send repeated start
#define TWCR_RACK 0xC4|(1<<TWIE) //receive byte and return ack to slave
#define TWCR_RNACK 0x84|(1<<TWIE) //receive byte and return nack to slave
#define TWCR_SEND 0x84|(1<<TWIE) //pokes the TWINT flag in TWCR and TWEN

/************************************************************************/
/* Display Functions                                                    */
/************************************************************************/

void DummyLoop(uint16_t count)
{
	//Each index1 loop takes approx 100us
	for ( uint16_t index1=0; index1<=count; index1=index1+1 ) {
		//Each index2 loop takes .5us (200 loops = 100us)
		for ( uint16_t index2=0; index2<=200; index2=index2+1 ){
			//Do nothing
		}
	}
}

void UpdateTWIDisplayState()
{
	switch(display_state) {
		case 0: //Start of a new display update
		TWCR = TWCR_START;  //send start condition
		display_state++;
		break;
		case 1: //Send address
		TWDR = 0x78; //Set TWI slave address
		TWCR = TWCR_SEND; //Send address
		display_state++;
		break;
		case 2: //Send Control Byte (Instruction: 0x80)
		TWDR = 0x80; //Instruction 0x80
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 3: //Send Instruction Byte (Set DDRAM Address: 0x80)
		TWDR = 0x80; //Set DDRAM Address: 0x80
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 4: //Send Control Byte (DDRAM Data: 0x40)
		TWDR = 0x40; //DDRAM Data: 0x40
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 5: //Send First Line Character
		TWDR = FirstLineStr[TWI_char_index];
		if(TWI_char_index<19) //If not last character
		{
			TWI_char_index++; //Increment index
		}
		else //If last character
		{
			TWI_char_index = 0; //Reset index for next line
			display_state++; //Move to next line state
		}
		TWCR = TWCR_SEND; //Set TWINT to send data
		break;
		case 6: //Send Third Line Character
		TWDR = ThirdLineStr[TWI_char_index];
		if(TWI_char_index<19) //If not last character
		{
			TWI_char_index++; //Increment index
		}
		else //If last character
		{
			TWI_char_index = 0; //Reset index for next line
			display_state++; //Send stop signal
		}
		TWCR = TWCR_SEND; //Set TWINT to send data
		break;
		case 7: //Send repeated start to reset display
		TWCR = TWCR_RSTART;
		display_state++;
		break;
		case 8: //Send address
		TWDR = 0x78; //Set TWI slave address
		TWCR = TWCR_SEND; //Send address
		display_state++;
		break;
		case 9: //Send Control Byte (Instruction: 0x80)
		TWDR = 0x80; //Instruction 0x80
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 10: //Send Instruction Byte (Set DDRAM Address: 0x80)
		TWDR = 0xC0; //Set DDRAM Address: 0xC0
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 11: //Send Control byte for DDRAM data
		TWDR = 0x40;
		TWCR = TWCR_SEND; //Set TWINT to send data
		TWI_char_index =0;
		display_state++;
		break;
		case 12: //Send Second Line Characters
		TWDR = SecondLineStr[TWI_char_index];
		if(TWI_char_index<19) //If not last character
		{
			TWI_char_index++; //Increment index
		}
		else //If last character
		{
			TWI_char_index = 0; //Reset index for next line
			display_state++; //Send stop signal
		}
		TWCR = TWCR_SEND; //Set TWINT to send data
		break;
		case 13: //Send Fourth Line Characters
		TWDR = FourthLineStr[TWI_char_index];
		if(TWI_char_index<19) //If not last character
		{
			TWI_char_index++; //Increment index
		}
		else //If last character
		{
			TWI_char_index = 0; //Reset index for next line
			display_state++; //Send stop signal
		}
		TWCR = TWCR_SEND; //Set TWINT to send data
		break;
		case 14: //Create Stop Condition
		TWCR = TWCR_STOP;//finish transaction
		display_state =0;
		break;
		/************************************************************************/
		/* Initialization States                                                */
		/************************************************************************/
		case 17: //Initialize Step One
		DummyLoop(400);//Wait 40ms for powerup

		TWCR = TWCR_START;
		display_state++;
		break;
		case 18:
		TWDR = 0x78; //Set TWI slave address
		TWCR = TWCR_SEND; //Send address
		display_state++;
		break;
		case 19: //Send control byte for instruction
		TWDR = 0x80; //Instruction 0x80
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 20: //Set Function Mode
		TWDR = 0x38;
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 21: //Send Stop and Wait
		TWCR = TWCR_STOP;//finish transaction
		DummyLoop(1);//Delay 100us
		TWCR = TWCR_START;//Start next transaction
		display_state++;
		break;
		case 22: //Send Slave Address
		TWDR = 0x78; //Set TWI slave address
		TWCR = TWCR_SEND; //Send address
		display_state++;
		break;
		case 23: //Send control byte for instruction
		TWDR = 0x80; //Instruction 0x80
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 24: //Turn on Display, Cursor, and Blink
		TWDR = 0x0C; //Instruction 0x80
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 25: //Send Stop and Wait
		TWCR = TWCR_STOP;//finish transaction
		DummyLoop(1);//Delay 100us
		TWCR = TWCR_START;//Start next transaction
		display_state++;
		break;
		case 26: //Send Slave Address
		TWDR = 0x78; //Set TWI slave address
		TWCR = TWCR_SEND; //Send address
		display_state++;
		break;
		case 27: //Send control byte for instruction
		TWDR = 0x80; //Instruction 0x80
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 28: //Clear Display
		TWDR = 0x01; //Instruction 0x01
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 29: //Send Stop and Wait
		TWCR = TWCR_STOP;//finish transaction
		DummyLoop(100);//Delay 10ms
		TWCR = TWCR_START;//Start next transaction
		display_state++;
		break;
		case 30: //Send Slave Address
		TWDR = 0x78; //Set TWI slave address
		TWCR = TWCR_SEND; //Send address
		display_state++;
		break;
		case 31: //Send control byte for instruction
		TWDR = 0x80; //Instruction 0x80
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 32: //Clear Display
		TWDR = 0x01; //Instruction 0x01
		TWCR = TWCR_SEND; //Set TWINT to send data
		display_state++;
		break;
		case 33: //Send Stop and Wait
		TWCR = TWCR_STOP;//finish transaction
		DummyLoop(1);//Delay 100us
		display_state=0;
		break;
		default:
		display_state = 0;
	}

}

/************************************************************************/
/* Define TWI Functions                                                 */
/************************************************************************/

ISR(TWI_vect)
{
	PORTD ^= (1<<PORTD2);
	//Read status register and mask out prescaler bits
	uint8_t status = TWSR & 0xF8;

	//Switch based on status of TWI interface
	switch(status)
	{
		case 0x08: //Start Condition Transmitted
		UpdateTWIDisplayState();
		break;
		case 0x10: //Repeat Start Condition Transmitted
		UpdateTWIDisplayState();
		break;
		case 0x18: //SLA+W transmitted and ACK received
		UpdateTWIDisplayState();
		break;
		case 0x20: //SLA+W transmitted and ACK NOT received
		//This is an error, do something application specific
		break;
		case 0x28: //Data byte transmitted and ACK received
		UpdateTWIDisplayState();
		break;
		case 0x30: //Data byte transmitted and ACK NOT received
		//This is an error, do something application specific
		break;
	}
	PORTD ^= (1<<PORTD2);
}



#endif /* DISPLAY_H_ */