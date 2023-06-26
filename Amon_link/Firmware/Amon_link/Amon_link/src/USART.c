/*
 * USART.c
 *
 * Created: 3/25/2023 10:33:00 PM
 *  Author: Tinta T.
 */ 
#include "../inc/USART.h"

// Variables
volatile uint8_t RecievedDataBuffer;
volatile uint8_t TransmitDataBuffer;
uint8_t Data_Count = 0;
uint8_t USART_DATA_RAW_Recieve[100] = {};

void USART_Init(unsigned int presc)
{
	UBRR0H = (unsigned char)(presc>>8);
	UBRR0L = (unsigned char)presc;
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0) | (1<<TXCIE0);
	UCSR0C = USART_MODE | USART_PARITY_MODE | USART_STOP_BIT | USART_DATA_BIT;
}

// USART Transmit
ISR(USART_TX_vect)
{
	// TRANSMIT...
}

// USART Receive
ISR(USART_RX_vect)
{
	RecievedDataBuffer = UDR0; // incoming data  UCSR0B

	// Read data and save in buffer. If sequence of dual f is detected the entire data is received
	if (USART_DATA_RAW_Recieve[Data_Count - 1] == 'f' && RecievedDataBuffer == 'f'){
		USART_DATA_RAW_Recieve[Data_Count] = RecievedDataBuffer; // save last byte
		Data_Count = 0;		// reset data counter for saving in array
		USART_RX_DATA_Decode(USART_DATA_RAW_Recieve);
	}
	else
	{
		// Save received data and increase counter
		USART_DATA_RAW_Recieve[Data_Count] = RecievedDataBuffer;
		Data_Count++;
	}
}

/* USART decode and communication */
void USART_RX_DATA_Decode(uint8_t data[100])
{
	uint8_t responsePacket[100] = {};
	
	if (data[0] == '0')			// LINK command
	{
		if (data[1] == '1')					//setup
		{
			uint8_t baud1 = (char)(USART_BAUD/100000);
			uint8_t baud2 = (char)((USART_BAUD-baud1*100000)/10000);
			uint8_t baud3 = (char)((USART_BAUD-baud1*100000-baud2*10000)/1000);
			uint8_t baud4 = (char)((USART_BAUD-baud1*100000-baud2*10000-baud3*1000)/100);
			uint8_t baud5 = (char)((USART_BAUD-baud1*100000-baud2*10000-baud3*1000-baud4*100)/10);
			uint8_t baud6 = (char)((USART_BAUD-baud1*100000-baud2*10000-baud3*1000-baud4*100-baud5*10)%10);
			
			responsePacket[0] = (char)(CODE_ID_LINK_RESPONSE/10+48);
			responsePacket[1] = (char)(CODE_ID_LINK_RESPONSE%10+48);
			responsePacket[2] = ',';
			responsePacket[3] = (char)(baud1+48);
			responsePacket[4] = (char)(baud2+48);
			responsePacket[5] = (char)(baud3+48);
			responsePacket[6] = (char)(baud4+48);
			responsePacket[7] = (char)(baud5+48);
			responsePacket[8] = (char)(baud6+48);
			responsePacket[9] = 'f';
			responsePacket[10] = 'f';
						
			USART_DATA_Transmit(responsePacket);
		}
		
		if (data[1] == '2')					// Errors (NOT PROGRAMED YET)
		{
			responsePacket[0] = (char)(CODE_ID_LINK_RESPONSE/10+48);
			responsePacket[1] = (char)(CODE_ID_LINK_RESPONSE%10+48);
			responsePacket[2] = ',';
			responsePacket[3] = '0';
			responsePacket[4] = '0';
			responsePacket[5] = 'f';
			responsePacket[6] = 'f';
			
			USART_DATA_Transmit(responsePacket);
		}
		
		if (data[1] == '3')					// Pair
		{
			
			responsePacket[0] = (char)(CODE_ID_LINK_RESPONSE/10+48);
			responsePacket[1] = (char)(CODE_ID_LINK_RESPONSE%10+48);
			responsePacket[2] = ',';
			responsePacket[3] = (char)(CODE_ID_PAIR);
			responsePacket[4] = 'f';
			responsePacket[5] = 'f';	
					
			USART_DATA_Transmit(responsePacket);
		}
	}
	else if (data[0] == '1')		// DRONE command
	{
		USART_DATA_Transmit(responsePacket);
	}
	else if (data[0] == '2')		// DRONE FLIGHT command
	{
		USART_DATA_Transmit(responsePacket);
	}
	else if(data[0] == '3')			// DRONE AFTER FLIGHT command
	{
		USART_DATA_Transmit(responsePacket);
	}
	else
	{
		// SEND TO DRONE
	}
}

void USART_DATA_Transmit(uint8_t data[100])
{
	for(int i = 0; i<100;i++){
		if (data[i] == 'f' && data[i+1] == 'f'){
			break;
		}else{
			while ( !( UCSR0A & (1<<UDRE0)) );		
			UDR0 = data[i]; // TransmitDataBuffer
		}
	}
}