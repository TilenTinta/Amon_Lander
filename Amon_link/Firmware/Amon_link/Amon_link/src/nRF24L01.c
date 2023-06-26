/*
 * nRF24L01.c
 *
 * Created: 06/03/2023 22:40:58
 *  Author: Tinta T.
 */ 


#include "../inc/nRF24L01.h"

void SPI_Init()
{
	// CS, MOSI, SCK output; MISO input -> Set in main
    // enable and set SPI
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
}

uint8_t SPI_Transmit(unsigned char data)
{
	SPDR = data; // load data in register
	while(!(SPSR & (1 << SPIF))); // Wait for transmission
	return SPDR;
}

void SPI_Transmit_Byte(uint8_t data, uint8_t CS_Pin)
{
	GPIO_PIN_Write(GPIO_PORT_B,CS_Pin,GPIO_LOW);
	SPI_Transmit(data);
	GPIO_PIN_Write(GPIO_PORT_B,CS_Pin,GPIO_HIGH);
}

uint8_t SPI_Receive()
{
	uint8_t data;
	data = SPDR; // load data in register
	while(!(SPSR & (1 << SPIF))); // Wait for receipsion
	return data;
}


uint8_t SPI_TransmitReceive(uint8_t data, uint8_t CS_Pin)
{
	_delay_us(10);
	GPIO_PIN_Write(GPIO_PORT_D,CS_Pin,GPIO_LOW); //CS low to enable device
	_delay_us(10);
    SPI_Transmit(0x55);
	_delay_us(10);
    uint8_t ret = SPI_Transmit(0xFF); 
	_delay_us(10);
	GPIO_PIN_Write(GPIO_PORT_D,CS_Pin,GPIO_HIGH); //CS low to disable device
	
	return ret;
}

void NRF24L01_Inti1()
{
	_delay_us(100);
	
	
		
}

void NRF24L01_Inti2()
{
	
	
}

uint8_t SPI_test(uint8_t CS_Pin)
{
	_delay_us(10);
	GPIO_PIN_Write(GPIO_PORT_D,CS_Pin,GPIO_LOW); //CS low to enable device
	_delay_us(10);
	SPI_Transmit(R_REGISTER + STATUS);
	_delay_us(10);
	uint8_t ret = SPI_Transmit(NOP);
	_delay_us(10);
	GPIO_PIN_Write(GPIO_PORT_D,CS_Pin,GPIO_HIGH); //CS low to disable device
	
	return ret;	
}

