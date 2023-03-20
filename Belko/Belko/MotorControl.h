/*
 * MotorControl.h
 *
 * Created: 3/20/2023 9:29:50 AM
 *  Author: ponti
 */ 


#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

/************************************************************************/
/* Motor Control                                                        */
/************************************************************************/
void SetMotorSpeed(uint8_t left, uint8_t right) {
	
	/* Configuring DC% */
	OCR0A = left;
	OCR0B = right;
}

#endif /* MOTORCONTROL_H_ */