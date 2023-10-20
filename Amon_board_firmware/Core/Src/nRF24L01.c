/*
 * nRF24L01.c
 *
 *  Created on: May 19, 2023
 *      Author: titit
 */

#include "nRF24L01.h"

/*
uint8_t NRF24L01_WriteRegister(SPI_HandleTypeDef *spiHandle, uint8_t radio, uint8_t reg, uint8_t *data){

	if(radio == 1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, SET);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
	}

	HAL_SPI_Transmit(&spiHandle, data, 1, 100);

	if(radio == 1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, RESET);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
	}

}
*/
