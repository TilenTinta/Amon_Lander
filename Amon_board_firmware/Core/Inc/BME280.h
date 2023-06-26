/*
 * BME280.h
 *
 *  Created on: Feb 23, 2023
 *      Author: Tinta T.
 */

#ifndef BME280_H_
#define BME280_H_

#include "stm32f4xx_hal.h"					/* For i2c communication */

/*
 * REGISTER DEFINES
 * (p.32, p.27...)
 *
 * Might have problem with HAL_OK, power cycle the device and try again
 * t_fine (datasheet) is value of temperature and it has to be read first (used in press and hum compensation)
 *
 */

#define ID_REG				0xD0		/* page 27 ; SDO -> GND: 0x76; SDO -> VDD: 0x77 (used 0x76) */
#define BME280_ID 			(0x76 << 1)	/* page 27 ; device ID (LSL -> 7bit addres) */
#define RESET				0xE0		/* page 27 ; for reset write value 0xB6, returns 0x00 */
#define RESET_VAL			0xB6

#define CTRL_HUM 			0xF2		/* page 27 ; set configurations data for humidity measuring */
#define STATUS 				0xF3		/* page 28 ; returns current status of device (two bits, both 0 is idle) */
#define CTRL_MEAS 			0xF4		/* page 28 ; set configurations data for pressure and temperature measuring */
#define CONFIG_BME280 		0xF5		/* page 30 ; set configurations data for device */

// Pressure
#define PRESS_MSB 			0xF7		/* page 30 ; pressure data */
#define PRESS_LSB 			0xF8
#define PRESS_XLSB 			0xF9

// Temperature
#define TEMP_MSB 			0xFA		/* page 30 ; temperature data */
#define TEMP_LSB 			0xFB
#define TEMP_XLSB			0xFC

// Humidity
#define HUM_MSB 			0xFD		/* page 30 ; humidity data */
#define HUM_LSB 			0xFE

// Compensation data
#define DIG_T1_1			0x88
#define DIG_T1_2			0x89
#define DIG_T2_1			0x8A
#define DIG_T2_2			0x8B
#define DIG_T3_1			0x8C
#define DIG_T3_2			0x8D
#define DIG_P1_1			0x8E
#define DIG_P1_2			0x8F
#define DIG_P2_1			0x90
#define DIG_P2_2			0x91
#define DIG_P3_1			0x92
#define DIG_P3_2			0x93
#define DIG_P4_1			0x94
#define DIG_P4_2			0x95
#define DIG_P5_1			0x96
#define DIG_P5_2			0x97
#define DIG_P6_1			0x98
#define DIG_P6_2			0x99
#define DIG_P7_1			0x9A
#define DIG_P7_2			0x9B
#define DIG_P8_1			0x9C
#define DIG_P8_2			0x9D
#define DIG_P9_1			0x9E
#define DIG_P9_2			0x9F

#define DIG_H1				0xA1
#define DIG_H2_1			0xE1
#define DIG_H2_2			0xE2
#define DIG_H3				0xE3
#define DIG_H4_1			0xE4
#define DIG_H4_2			0xE5	/* 3:0 */
#define DIG_H5_1			0xE5	/* 7:4 */
#define DIG_H5_2			0xE6	/* 11:8 */

/*
 * STRUCTS
 */

typedef struct {

	I2C_HandleTypeDef *i2cHandle;	/* i2c Handle */

	uint32_t Temp_C;		/* Temperature value in Celsius */

	uint32_t Hum_Perc;		/* Humidity value in percents */

	uint32_t Press_Pa;		/* Pressure value in Pascals */

	int32_t t_fine;

	/* Calibration data from sensor */
	unsigned short dig_T1;
	signed short dig_T2;
	signed short dig_T3;

	unsigned short dig_P1;
	signed short dig_P2;
	signed short dig_P3;
	signed short dig_P4;
	signed short dig_P5;
	signed short dig_P6;
	signed short dig_P7;
	signed short dig_P8;
	signed short dig_P9;

	unsigned char dig_H1;
	signed short dig_H2;
	unsigned char dig_H3;
	signed short dig_H4;
	signed short dig_H5;
	signed char dig_H6;

} BME280;


/*
 * FUNCTIONS
 */

// Read device ID register
uint8_t BME280_ReadDeviceID(BME280 *dev, I2C_HandleTypeDef *i2cHandle);

// Init device
uint8_t BME280_Init(BME280 *dev, I2C_HandleTypeDef *i2cHandle);

// Read calibration data from sensor flash
uint8_t BME280_ReadCalibData(BME280 *dev, I2C_HandleTypeDef *i2cHandle);

// Read temperature
uint8_t BME280_ReadTemperature(BME280 *dev, I2C_HandleTypeDef *i2cHandle);

// Read humidity
uint8_t BME280_ReadHumidity(BME280 *dev, I2C_HandleTypeDef *i2cHandle);

// Read pressure
uint8_t BME280_ReadPressure(BME280 *dev, I2C_HandleTypeDef *i2cHandle);

// Read all measurements
uint8_t BME280_ReadAllData(BME280 *dev, I2C_HandleTypeDef *i2cHandle);

// Reset device
uint8_t BME280_Reset(BME280 *dev, I2C_HandleTypeDef *i2cHandle);

#endif /* BME280_H_ */
