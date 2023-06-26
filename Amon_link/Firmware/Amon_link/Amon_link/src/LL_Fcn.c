/*
 * LL_Fcn.c
 *
 * Created: 27/02/2023 23:06:47
 *  Author: Tinta T.
 */ 

#include "../inc/LL_Fcn.h"

/* Set pin as input/output */
void GPIO_PIN_SETUP(char port, uint8_t pinNum, char fcn){
	
	switch(port){
		case 1:
			DDRB = (fcn<<pinNum);
		case 2:
			DDRC = (fcn<<pinNum);
		case 3:
			DDRD = (fcn<<pinNum);
		default:
			break;
	}
}

/* Set pin to high/low */
void GPIO_PIN_Write(char port, uint8_t pinNum, char state){
	
	switch(port){
		case 1:
			(state) ? (PORTB |= (1 << pinNum)) : (PORTB &= ~(1 << pinNum));
		case 2:
			(state) ? (PORTC |= (1 << pinNum)) : (PORTC &= ~(1 << pinNum));
		case 3:
			(state) ? (PORTD |= (1 << pinNum)) : (PORTD &= ~(1 << pinNum));
		default:
		break;
	}
}

/* Toggle pin value */
void GPIO_PIN_Toggle(char port, uint8_t pinNum){
		
	switch(port){
		case 1:
			PORTB ^= (1 << pinNum);
		case 2:
			PORTC ^= (1 << pinNum);
		case 3:
			PORTD ^= (1 << pinNum);
		default:
		break;
	}
}

/* Toggle pin value */
uint8_t GPIO_PIN_Read(char port, uint8_t pinNum){
	
	switch(port){
		case 1:
			return ((PINB & (1 << pinNum)) >> pinNum);
		case 2:
			return ((PINC & (1 << pinNum)) >> pinNum);
		case 3:
			return ((PIND & (1 << pinNum)) >> pinNum);
		default:
		return 0;
	}
}