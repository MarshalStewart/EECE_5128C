/*
 * Belko.cpp
 *
 * Created: idk
 * Author : Marshal and Kelsey
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

#include "Display.h"
#include "MotorControl.h"

/* Analog Sensor Values */
volatile uint8_t left_sensor = 0;
volatile uint8_t ir_beacon_sensor = 0;
volatile uint8_t right_sensor = 0;
volatile uint8_t photo_res_sensor = 0;

/* Digital Sensor Values */
volatile uint8_t bump_sensor = 0;

int main(void)
{
	//**************************************************************************************
	//Configure I/O
	//**************************************************************************************
	// PORTB
	// PB3 (out): PWM Timer 0A, Left Motor
	// PB4 (out): PWM Timer 0B, Right Motor
	DDRB = (1<<PORTB4)|(1<<PORTB3);
	
	// PORTC
	// PC0 (in): ADC0, IR Prox Sensor Left
	// PC1 (in): ADC1, IR Beacon Sensor
	// PC2 (in): ADC2, IR Prox Sensor Right
	// PC3 (in): ADC3, Photor Resistor
	// PC4 (out): SDA, I2C Display
	// PC5 (out): SCL, I2C Display
	// PC6 (in): RESET
	DDRC = (0<<PORTC6)|(1<<PORTC5)|(1<<PORTC4)|(0<<PORTC3)|(0<<PORTC2)|(0<<PORTC1)|(0<<PORTC0);
	
	// PORTD
	// PD0 (in): Digital, Line Sensor
	// PD1 (out): Digital, Debug 0
	// PD2 (out): Digital, Debug 1
	// PD3 (out): Digital, Alarm Relay Control
	// PD4 (in): INT0, Bump Sensor (Hardware Debounced)
	DDRD = (0<<PORTD4)|(1<<PORTD3)|(1<<PORTD2)|(1<<PORTD1)|(0<<PORTD0);
	
	//**************************************************************************************
	//Configure Timer 0 (Motor Control Timer) Pg 84
	//**************************************************************************************
	//Setup Fast PWM Mode for Channel A and B
	//**************************************************************************************
	// COM0A1 COM0A0 => Clear OC0A on compare match, set OC0A at BOTTOM (non-inverting mode)
	// COM0B1 COM0B0 => Clear OC0B on compare match, set OC0B at BOTTOM (non-inverting mode)
	// WGM02 WGM01 WGM00 => Fast PWM, TOP 0xFF, Update of OCRx at 0x00, TOV Flag set on 0xFF
	// CS02 CS01 CS00 => ClkIO/1024 (15 kHz)
	TCCR0A = (1<<COM0B1)|(0<<COM0B0)|(1<<COM0A1)|(0<<COM0A0)|(1<<WGM01)|(1<<WGM00);
	TCCR0B = (0<<WGM02)|(1<<CS02)|(0<<CS01)|(1<<CS00);

	// Initialize Motor Speeds to zero
	OCR0A = 0;
	OCR0B = 0;

	//**************************************************************************************
	//Configure Timer 1 (TWI and ADC)
	//**************************************************************************************
    //No Pin Toggles, CTC Mode
    TCCR1A = (0<<COM1A1)|(1<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM01);
    //CTC Mode, N=64
    TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10);
    //Enable Channel A Match Interrupt
    TIMSK1 = (0<<ICIE1)|(0<<OCIE1B)|(1<<OCIE1A)|(0<<TOIE1);
    //Setup output compare for channel A
    OCR1A = 24999; //OCR1A = 16Mhz/N*Deltat-1 = 16Mhz/64*.1-1
	
	///* Set Timer 1 Pre-scalar */
	//TCCR1B = (1<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
	///* Set Compare 1 A */
	//OCR1AH = 0x09;
	//OCR1AL = 0xC3;
	
	//**************************************************************************************
	//Configure TWI
	//**************************************************************************************
    //Configure TWI module (no interrupts)
    //Configure Bit Rate (TWBR)
    TWBR = 50;
    //Configure TWI Interface (TWCR)
    //TWINT: 0 - Interrupt Flag
    //TWEA: 0 - No acknowledge Bit
    //TWSTA: 0 - Start Condition Bit
    //TWSTO: 0 - Stop Condition
    //TWEN: 1 - Enable TWI Interface
    //TWIE: 0 - Disable Interrupt
    TWCR = (0<<TWINT)|(0<<TWEA)|(0<<TWSTO)|(0<<TWWC)|(1<<TWEN)|(1<<TWIE);
    //Configure TWI Status Register (TWSR)
    //TWS7-TWS3: 00000 - Don't change status
    //TWPS1-TWPS0: 10 - Prescaler Value = 64
    TWSR = (0<<TWS7)|(0<<TWS6)|(0<<TWS5)|(0<<TWS4)|(0<<TWS3)|(1<<TWPS1)|(0<<TWPS0);

	//Initialize Display
	display_state=17;//Set state machine to initialization section
	UpdateTWIDisplayState();//Run state machine
	
	//**************************************************************************************
	//Configure ADC
	//**************************************************************************************
	// ADC7-0 in low word, ADC9-8 are in high word
	ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0)|(1<<ADLAR); // Start at ADC0
	ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	DIDR0 = (1<<ADC1D);
	
	//*********************************************************************
	//Configure External Interrupt 0 (Bump Sensor)
	//*********************************************************************
	EICRA = (1<<ISC01)|(0<<ISC00); // Detect falling edge
	EIMSK = (1<<INT0);//Enable INT0
	
    sei();
		
    /* Replace with your application code */
    while (1) {}
}

ISR(ADC_vect)
{
	static uint8_t adc_index = 0;
	const uint8_t adcmux_init = (0<<REFS1)|(1<<REFS0)|(1<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
	uint8_t adc_read = 0;
	
	// Always read ADC
	adc_read = ADCL >> 6;
	adc_read |= (ADCH << 2);
	
	// Handle appropriate signals
	switch(adc_index)
	{
		case 0: // ADC0
			left_sensor = adc_read;
			break;
		case 1: // ADC1
			ir_beacon_sensor = adc_read;
			break;
		case 2: // ADC2
			right_sensor = adc_read;
			break;
		case 3: // ADC3
			photo_res_sensor = adc_read;
			break;
	}
	
	/* Toggle ADC Conversion Complete Breadcrumb */
	PORTD ^= (1<<PORTD1);

	// Setting up next conversion
	adc_index++;

	if (adc_index > 3)
	{
		adc_index = 0;
	}
	
	ADCSRB = (adcmux_init & 0xF0) | adc_index;

}

ISR(INT0_vect)
{
	// TODO: Write Bump Routine
	
	bump_sensor = (PIND >> PIND4) & 0b1;
}

ISR(TIMER1_COMPA_vect)
{
	//static uint8_t bcount = 0; // Number of iterations in reverse
	const uint8_t threshold = 200;
	
	ADCSRA |= (1<<ADSC); //Start Conversion
	/* Mux to other ADC */	
	ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<MUX2)|(0<<MUX1)|(1<<MUX0)|(1<<ADLAR); // ADC0 and ADC 1
		
	/* Display frame rate: 500ms (Based on 10ms Timer 1) */
	if (lcd_counter > 50) {
		//Start New LCD Screen Update
		display_state = 0;
		if(display_state==0) {
			UpdateTWIDisplayState();
		}
		
		lcd_counter = 0;
	} else { lcd_counter++; }
		
	//**************************************************************************************
	// Motor Control Logic
	//**************************************************************************************
	
	// TODO: PID Controller here
	
	// Obstacle Avoidance
	if (left_sensor < threshold)
	{
		SetMotorSpeed(200, 50);
	}
	else if (right_sensor < threshold)
	{
		SetMotorSpeed(50, 200);
	}
	else // Move forward
	{
		SetMotorSpeed(200, 200);
	}
	
	//**************************************************************************************
	// Write to Display here
	//**************************************************************************************
	snprintf((char *)FirstLineStr, 21, "M1:%u", left_sensor);
	snprintf((char *)SecondLineStr, 21, "M2:%u", right_sensor);
	snprintf((char *)ThirdLineStr, 21, "Button: %-13s", (bump_sensor ? "On" : "Off"));	
	
}

