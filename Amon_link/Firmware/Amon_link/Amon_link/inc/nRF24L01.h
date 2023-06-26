/*
 * nRF24L01.h
 *
 * Created: 11/03/2023 11:53:36
 *  Author: titit
 */ 


#ifndef NRF24L01_H_
#define NRF24L01_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <xc.h>
#include <util/delay.h>
#include "../inc/LL_Fcn.h"

#define R_REGISTER			0x00
#define W_REGISTER			0x20
#define R_RX_PAYLOAD		0x61
#define R_TX_PAYLOAD		0xA0
#define FLUSH_RX			0xE1
#define FLUSH_TX			0xE2
#define REUSE_TX_PL			0xE3
#define ACTIVATE			0x50
#define R_RX_PL_WID			0x60
#define W_ACK_PAYLOAD		0xA8 // last tree bits can be 000 to 101
#define W_TX_PAYLOAD_NOACK	0xB0 // ?? datasheet only three bits
#define NOP					0xFF

#define CONFIG				0x00
#define EN_AA				0x01
#define EN_RXADDR			0x02
#define SETUP_AW			0x03
#define SETUP_RETR			0x04
#define RF_CH				0x05
#define RF_SETUP			0x06
#define STATUS				0x07
#define OBSERVE_TX			0x08
#define CD					0x09
#define RX_ADDR_P0			0x0A
#define RX_ADDR_P1			0x0B
#define RX_ADDR_P2			0x0C
#define RX_ADDR_P3			0x0D
#define RX_ADDR_P4			0x0E
#define RX_ADDR_P5			0x0F
#define TX_ADDR				0x10
#define RX_PW_P0			0x11
#define RX_PW_P1			0x12
#define RX_PW_P2			0x13
#define RX_PW_P3			0x14
#define RX_PW_P4			0x15
#define RX_PW_P5			0x16
#define FIFO_STATUS			0x17
#define DYNPD				0x1C
#define FEATURE				0x1D

void SPI_Init();
uint8_t SPI_Transmit(uint8_t data);
void NRF24L01_Transmit_Byte(uint8_t data, uint8_t CS_Pin);
uint8_t SPI_Receive();
uint8_t SPI_TransmitReceive(uint8_t data, uint8_t CS_Pin);
void NRF24L01_Inti1();
void NRF24L01_Inti2();
uint8_t SPI_test(uint8_t CS_Pin);


#endif /* NRF24L01_H_ */