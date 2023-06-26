
#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>

int8_t VL53L1_WriteMulti(VL53L1_DEV *dev, uint16_t reg, uint8_t *data, uint32_t lenght)
{

	return HAL_I2C_Mem_Write (dev -> i2cHandle, DEVICE_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, data, lenght, 100);
}

int8_t VL53L1_ReadMulti(VL53L1_DEV *dev, uint16_t reg, uint8_t *data, uint32_t lenght)
{

	return HAL_I2C_Mem_Read (dev -> i2cHandle, DEVICE_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, data, lenght, 100);
}

int8_t VL53L1_WrByte(VL53L1_DEV *dev, uint16_t reg, uint8_t data)
{

	return HAL_I2C_Mem_Write (dev -> i2cHandle, DEVICE_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, &data, 1, 100);
}

int8_t VL53L1_WrWord(VL53L1_DEV *dev, uint16_t reg, uint16_t data)
{
	uint8_t i2cBuff[2] = {0};

	i2cBuff[0] = (data >> 8) & 0xFF;
	i2cBuff[1] = data & 0xFF;

	return HAL_I2C_Mem_Write (dev -> i2cHandle, DEVICE_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, i2cBuff, 2, 100);
}

int8_t VL53L1_WrDWord(VL53L1_DEV *dev, uint16_t reg, uint32_t data)
{
	uint8_t i2cBuff[4] = {0};

	i2cBuff[0] = (data >> 24) & 0xFF;
	i2cBuff[1] = (data >> 16) & 0xFF;
	i2cBuff[2] = (data >> 8)  & 0xFF;
	i2cBuff[3] = data & 0xFF;

	return HAL_I2C_Mem_Write (dev -> i2cHandle, DEVICE_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, i2cBuff, 4, 100);
}

int8_t VL53L1_RdByte(VL53L1_DEV *dev, uint16_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read (dev -> i2cHandle, DEVICE_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, data, 1, 100);
}

int8_t VL53L1_RdWord(VL53L1_DEV *dev, uint16_t reg, uint16_t *data)
{
	uint8_t i2cBuff[2] = {0};
	HAL_StatusTypeDef status = 0;

	status = HAL_I2C_Mem_Read (dev -> i2cHandle, DEVICE_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, i2cBuff, 2, 100);
	*data = ((uint16_t)i2cBuff[0] << 8) | (uint16_t)i2cBuff[1];

	return status;
}

int8_t VL53L1_RdDWord(VL53L1_DEV *dev, uint16_t reg, uint32_t *data)
{
	uint8_t i2cBuff[4] = {0};
	HAL_StatusTypeDef status = 0;

	HAL_I2C_Mem_Read (dev -> i2cHandle, DEVICE_ADDRESS, reg, I2C_MEMADD_SIZE_16BIT, i2cBuff, 4, 100);
	*data = ((uint32_t)i2cBuff[0] << 24) | ((uint32_t)i2cBuff[1] << 16) | ((uint32_t)i2cBuff[2] << 8) | (uint32_t)i2cBuff[3];

	return status;
}

int8_t VL53L1_WaitMs(VL53L1_DEV *dev, int32_t wait_ms)
{
	HAL_Delay(wait_ms/1000);

	return 0;
}
