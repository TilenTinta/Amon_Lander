/*
 * nRF24L01.h
 *
 *  Created on: May 19, 2023
 *      Author: titit
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "stm32f4xx_hal.h"					/* For i2c communication */
#include "DroneData.h"

/*
 * REGISTER MAP
 */
#define CONFIG_NRF24L01				0x00
#define EN_AA						0x01
#define EN_RXADDR					0x02
#define SETUP_AW					0x03
#define SETUP_RETR					0x04
#define RF_CH						0x05
#define RF_SETUP					0x06
#define STATUS						0x07
#define OBSERVE_TX					0x08
#define CD							0x09
#define RX_ADDR_P0					0x0A
#define RX_ADDR_P1					0x0B
#define RX_ADDR_P2					0x0C
#define RX_ADDR_P3					0x0D
#define RX_ADDR_P4					0x0E
#define RX_ADDR_P5					0x0F
#define TX_ADDR						0x10
#define RX_PW_P0					0x11
#define RX_PW_P1					0x12
#define RX_PW_P2					0x13
#define RX_PW_P3					0x14
#define RX_PW_P4					0x15
#define RX_PW_P5					0x16
#define FIFO_STATUS					0x17
#define DYNPD						0x1C
#define FEATURES					0x1D

// variables to choose witch radio to use
#define RADIO1						1
#define RADIO2						2



typedef struct {

	SPI_HandleTypeDef *spiHandle;

} NRF24L01;


/* FUNCTIONS */

// Write
uint8_t NRF24L01_WriteRegister(NRF24L01 *dev, uint8_t radio, uint8_t reg, uint8_t *data);



#endif /* INC_NRF24L01_H_ */
