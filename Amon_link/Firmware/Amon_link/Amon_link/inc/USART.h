/*
 * USART.h
 *
 * Created: 25/03/2023 22:29:21
 *  Author: titit
 */ 


#ifndef USART_H_
#define USART_H_

#include <avr/interrupt.h>
#include <avr/io.h>

// USART
#define USART_BAUD			9600
#define uC_FREQ				16000000UL
#define USART_PRESC			((uC_FREQ/(USART_BAUD * 16UL)) - 1)

#define USART_MODE			(0<<UMSEL00)
#define USART_PARITY_MODE	(0<<UPM00)
#define USART_STOP_BIT		(0<<USBS0)
#define USART_DATA_BIT		(3<<UCSZ00)

#define RX_IRQ_DONE			(1<<RXCIE0)
#define DATA_EMPTY_REG		(1<<UDRIE0)

// Defines for communication codes
#define CODE_ID_NULL			00		// reserved
#define CODE_ID_LINK_SETUP		01		// Amon link settings configuration
#define CODE_ID_LINK_ERROR		02		// Amon link errors report
#define CODE_ID_PAIR			03		// Pair status link <-> drone
#define CODE_ID_LINK_RESPONSE	04		// Code of response packet
#define CODE_ID_AMON_SETUP		11		// Amon configure parameters
#define CODE_ID_AMON_ERROR		12		// errors on craft
#define CODE_ID_AMON_PARAMETERS 13		// list current parameters
#define CODE_ID_AMON_RESPONSE	14		// Code of response packet
#define CODE_ID_AMON_ARM		20		// Arm craft before flight
#define CODE_ID_AMON_FLY_DATA	21		// Real time fly data
#define CODE_ID_AMON_LAND		30		// Status of craft after flight

// Functions
void USART_Init(unsigned int presc);
uint8_t USART_Receive();
void USART_RX_DATA_Decode();
void USART_DATA_Transmit(uint8_t data[100]);

#endif /* USART_H_ */