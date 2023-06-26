 
#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

#include "vl53l1_types.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define DEVICE_ADDRESS		0x52			/* User added */

typedef struct {

	I2C_HandleTypeDef *i2cHandle;	/* i2c Handle */

} VL53L1_DEV;


int8_t VL53L1_WriteMulti(VL53L1_DEV *dev, uint16_t index, uint8_t *data, uint32_t count);

int8_t VL53L1_ReadMulti(VL53L1_DEV *dev, uint16_t index, uint8_t *data,uint32_t count);

int8_t VL53L1_WrByte(VL53L1_DEV *dev, uint16_t index, uint8_t data);

int8_t VL53L1_WrWord(VL53L1_DEV *dev, uint16_t index, uint16_t data);

int8_t VL53L1_WrDWord(VL53L1_DEV *dev, uint16_t index, uint32_t data);

int8_t VL53L1_RdByte(VL53L1_DEV *dev, uint16_t index, uint8_t *data);

int8_t VL53L1_RdWord(VL53L1_DEV *dev, uint16_t index, uint16_t *data);

int8_t VL53L1_RdDWord(VL53L1_DEV *dev, uint16_t index, uint32_t *data);

int8_t VL53L1_WaitMs(VL53L1_DEV *dev, int32_t wait_ms);

#ifdef __cplusplus
}
#endif

#endif
