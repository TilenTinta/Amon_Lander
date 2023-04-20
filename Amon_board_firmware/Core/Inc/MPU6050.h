/*
 * MPU6050.h
 *
 *  Created on: Apr 8, 2023
 *      Author: Tinta T.
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f4xx_hal.h"					/* For i2c communication */

/*
 *		REGISTER MAP
 */

#define SELF_TEST_X 				0x0D	/* page 9 - 11 */
#define SELF_TEST_Y 				0x0E
#define SELF_TEST_Z 				0x0F
#define SELF_TEST_A 				0x10

#define SMPRT_DIV					0x19	/* page 11 - 12 */

#define CONFIG_MPU6050						0x1A	/* page 13 ; FSYNC -> GND (unused), */
#define GYRO_CONFIG					0x1B	/* page 14 ; FS_SEL set to 250dps for self-test */
#define ACCEL_CONFIG				0x1C	/* page 15 ; ASF_SEL set to 8g for self-test */

#define FIFO_EN						0x23	/* page 16 ; enable data to be written in FIFO register */
#define I2C_MST_CTRL				0x24	/* page 17 ; used for sensor to act as master */
#define I2C_SLV0_ADDR				0x25	/* page 19 - 21 ; sensor communicate with its slave 0 */
#define I2C_SLV0_REG				0x26
#define I2C_SLV0_CTRL				0x27
#define I2C_SLV1_ADDR				0x28	/* page 22 ; sensor communicate with its slave 1 */
#define I2C_SLV1_REG				0x29
#define I2C_SLV1_CTRL				0x2A
#define I2C_SLV2_ADDR				0x2B	/* page 22 ; sensor communicate with its slave 2 */
#define I2C_SLV2_REG				0x2C
#define I2C_SLV2_CTRL				0x2D
#define I2C_SLV3_ADDR				0x2E	/* page 22 ; sensor communicate with its slave 3 */
#define I2C_SLV3_REG				0x2F
#define I2C_SLV3_CTRL				0x30
#define I2C_SLV4_ADDR				0x31	/* page 23 ; sensor communicate with its slave 4 - special options */
#define I2C_SLV4_REG				0x32
#define I2C_SLV4_DO					0x33
#define I2C_SLV4_CTRL				0x34
#define I2C_SLV4_DI					0x35
#define I2C_MST_STATUS				0x36	/* page 25 ; unused */
#define INT_PIN_CFG					0x27	/* page 26 ; unused */
#define INT_ENABLE					0x38	/* page 27 */
#define INT_STATUS					0x3A	/* page 28 */

#define ACCEL_XOUT_H				0x3B	/* page 29 ; store acceleration values */
#define ACCEL_XOUT_L				0x3C
#define ACCEL_YOUT_H				0x3D
#define ACCEL_YOUT_L				0x3E
#define ACCEL_ZOUT_H				0x3F
#define ACCEL_ZOUT_L				0x40
#define TEMP_OUT_H					0x41	/* page 30 ; store temperature values, temp (C)=TEMP_OUT(signed)/340+36.53 */
#define TEMP_OUT_L					0x42
#define GYRO_XOUT_H					0x43	/* page 31 ; store gyroscope values */
#define GYRO_XOUT_L					0x44
#define GYRO_YOUT_H					0x45
#define GYRO_YOUT_L					0x46
#define GYRO_ZOUT_H					0x47
#define GYRO_ZOUT_L					0x48

#define EXT_SENS_DATA_00			0x49	/* page 32,33 ; store data from slave devices, unused */
#define EXT_SENS_DATA_01			0x4A
#define EXT_SENS_DATA_02			0x4B
#define EXT_SENS_DATA_03			0x4C
#define EXT_SENS_DATA_04			0x4D
#define EXT_SENS_DATA_05			0x4E
#define EXT_SENS_DATA_06			0x4F
#define EXT_SENS_DATA_07			0x50
#define EXT_SENS_DATA_08			0x51
#define EXT_SENS_DATA_09			0x52
#define EXT_SENS_DATA_10			0x53
#define EXT_SENS_DATA_11			0x54
#define EXT_SENS_DATA_12			0x55
#define EXT_SENS_DATA_13			0x56
#define EXT_SENS_DATA_14			0x57
#define EXT_SENS_DATA_15			0x58
#define EXT_SENS_DATA_16			0x59
#define EXT_SENS_DATA_17			0x5A
#define EXT_SENS_DATA_18			0x5B
#define EXT_SENS_DATA_19			0x5C
#define EXT_SENS_DATA_20			0x5D
#define EXT_SENS_DATA_21			0x5E
#define EXT_SENS_DATA_22			0x5F
#define EXT_SENS_DATA_23			0x60

#define I2C_SLV0_DO					0x63	/* page 34 ; unused */
#define I2C_SLV1_DO					0x64	/* page 34 ; unused */
#define I2C_SLV2_DO					0x65	/* page 35 ; unused */
#define I2C_SLV3_DO					0x66	/* page 35 ; unused */
#define I2C_MST_DELAY_CTRL			0x67	/* page 36 ; unused */

#define SIGNAL_PATH_RESET			0x68	/* page 37 ; reset values like power up state */
#define USER_CTRL					0x6A	/* page 38 ; unused, master control */
#define PWR_MGMT_1					0x6B	/* page 40 ; clock, mode... */
#define PWR_MGMT_2					0x6C
#define FIFO_COUNT_H				0x72	/* page 43 ; number of bztes in fifo buffer */
#define FIFO_COUNT_L				0x73
#define FIFO_R_W					0x74	/* page 44 ; FIFO buffer */

#define WHO_AM_I					0x75	/* page 45 ; sensor ID register, default value 0x68 */
#define MPU6050_ID					(0x68 << 1)	/* Device ID */


/*
 *		STRUCTS
 */

typedef struct {

	I2C_HandleTypeDef *i2cHandle;	/* i2c Handle */

	float ACCEL_X;	/* acceleration x value */

	float ACCEL_Y;	/* acceleration x value */

	float ACCEL_Z;	/* acceleration x value */

	float GYRO_X;	/* gyroscope x value */

	float GYRO_Y;	/* gyroscope y value */

	float GYRO_Z;	/* gyroscope z value */

	float Temp_C;	/* gyroscope x value */

} MPU6050;


/*
 *		FUNCTIONS
 */

// LL read one register
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050 *dev, uint8_t reg, uint8_t *data);

// LL read multiple register
HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050 *dev, uint8_t reg, uint8_t *data, uint8_t lenght);

// LL write one register
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050 *dev, uint8_t reg, uint8_t *data);

// Read device ID register
uint8_t MPU6050_ReadDeviceID(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle);


#endif /* MPU6050_MPU6050_H_ */
