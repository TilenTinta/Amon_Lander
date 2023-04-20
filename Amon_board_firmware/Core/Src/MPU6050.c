/*
 * MPU6050.c
 *
 *  Created on: Apr 8, 2023
 *      Author: Tinta T.
 */
#include "MPU6050.h"

uint8_t MPU6050_ReadDeviceID(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t regData;

	status = MPU6050_ReadRegister(dev, WHO_AM_I, &regData);

	if (status != HAL_OK){
		return 255;	// NOK
	}
	else
	{
		if (regData != 0x68)
		{
			return 1; // OK
		}
		else
		{
			return 0; // OK
		}
	}
}

/*
uint8_t MPU6050_Init(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;

	status = MPU6050_WriteRegister(MPU6050 *dev, reg, data);

	if (status != HAL_OK){
		return 255;
	}else{
		return 0;
	};
}
*/


//* LL functions *//
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read (dev -> i2cHandle, MPU6050_ID, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
};

HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050 *dev, uint8_t reg, uint8_t *data, uint8_t lenght){
	return HAL_I2C_Mem_Read (dev -> i2cHandle, MPU6050_ID, reg, I2C_MEMADD_SIZE_8BIT, data, lenght, 100);
};

HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Write (dev -> i2cHandle, MPU6050_ID, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
};
