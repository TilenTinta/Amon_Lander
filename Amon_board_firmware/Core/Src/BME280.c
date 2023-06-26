/*
 * BME280.c
 *
 *  Created on: Feb 23, 2023
 *      Author: Tinta T.
 */

#ifndef BME280_C_
#define BME280_C_

#include "BME280.h"

/*
 * Private functions
 */

HAL_StatusTypeDef BME280_ReadRegister(BME280 *dev, uint8_t reg, uint8_t *data);						// Read one register from device

HAL_StatusTypeDef BME280_ReadRegisters(BME280 *dev, uint8_t reg, uint8_t *data, uint8_t lenght);	// Read multiple registers from device

HAL_StatusTypeDef BME280_WriteRegister(BME280 *dev, uint8_t reg, uint8_t data);						// Write register to device

int32_t BME280_TemperatureCompesation(BME280 *dev, int32_t RawTemp);								// Compensate raw temperature value with compensation values from sensor

uint32_t BME280_HumidityCompesation(BME280 *dev, int32_t RawHum);									// Compensate raw humidity value with compensation values from sensor

uint32_t BME280_PressureCompesation(BME280 *dev, int32_t RawPress);									// Compensate raw pressure value with compensation values from sensor


uint8_t BME280_ReadDeviceID(BME280 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t reg_data;

	status = BME280_ReadRegister(dev, ID_REG, &reg_data);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}
	else
	{
		if (reg_data == 0x76){
			return 2; // NOK
		}
		else
		{
			return 0; // OK
		}
	}
}


uint8_t BME280_Reset(BME280 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status = 0;

	// Set al to 0
	dev->Temp_C = 0;
	dev->Hum_Perc = 0;
	dev->Press_Pa = 0;
	dev->t_fine = 0;

	dev->dig_T1 = 0;
	dev->dig_T2 = 0;
	dev->dig_T3 = 0;

	dev->dig_H1 = 0;
	dev->dig_H2 = 0;
	dev->dig_H3 = 0;
	dev->dig_H4 = 0;
	dev->dig_H5 = 0;
	dev->dig_H6 = 0;

	dev->dig_P1 = 0;
	dev->dig_P2 = 0;
	dev->dig_P3 = 0;
	dev->dig_P4 = 0;
	dev->dig_P5 = 0;
	dev->dig_P6 = 0;
	dev->dig_P7 = 0;
	dev->dig_P8 = 0;
	dev->dig_P9 = 0;

	status = BME280_WriteRegister(dev, RESET, RESET_VAL);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}
	else
	{
		return 0; // OK
	}
}


uint8_t BME280_Init(BME280 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;

	// Humidity; Set value: 011 = oversampling x4
	static const uint8_t ctrl_humData = 0x03;
	status = BME280_WriteRegister(dev, CTRL_HUM, ctrl_humData);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	// Temp, Press, Mode; Set value: 01101111 = oversampling x4, oversampling x4, normal mode
	static const uint8_t scrl_meadData = 0x6F;
	status = BME280_WriteRegister(dev, CTRL_MEAS, scrl_meadData);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	// Standbay, time IIR filter, 3-wire SPI, Mode; Set value: 0000010 = standbay 0.5ms, IIR 2, off SPI
	static const uint8_t confData = 0x02;
	status = BME280_WriteRegister(dev, CONFIG_BME280, confData);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}
	else
	{
		return 0; // OK
	}
}


uint8_t BME280_ReadCalibData(BME280 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t CalibData[25] = {};

	status = BME280_ReadRegisters(dev, DIG_T1_1, CalibData, 25);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	dev -> dig_T1 = (uint16_t)(CalibData[1] << 8) | (CalibData[0]);
	dev -> dig_T2 = (uint16_t)(CalibData[3] << 8) | (CalibData[2]);
	dev -> dig_T3 = (uint16_t)(CalibData[5] << 8) | (CalibData[4]);
	dev -> dig_P1 = (uint16_t)(CalibData[7] << 8) | (CalibData[6]);
	dev -> dig_P2 = (uint16_t)(CalibData[9] << 8) | (CalibData[8]);
	dev -> dig_P3 = (uint16_t)(CalibData[11] << 8) | (CalibData[10]);
	dev -> dig_P4 = (uint16_t)(CalibData[13] << 8) | (CalibData[12]);
	dev -> dig_P5 = (uint16_t)(CalibData[15] << 8) | (CalibData[14]);
	dev -> dig_P6 = (uint16_t)(CalibData[17] << 8) | (CalibData[16]);
	dev -> dig_P7 = (uint16_t)(CalibData[19] << 8) | (CalibData[18]);
	dev -> dig_P8 = (uint16_t)(CalibData[21] << 8) | (CalibData[20]);
	dev -> dig_P9 = (uint16_t)(CalibData[23] << 8) | (CalibData[22]);
	dev -> dig_H1 = CalibData[24];


	uint8_t CalibData2[7];
	status = BME280_ReadRegisters(dev, DIG_H2_1, CalibData2, 7);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	dev -> dig_H2 = ((uint16_t)CalibData2[1] << 8) | ((uint16_t)CalibData2[0]);
	dev -> dig_H3 = CalibData2[2];
	dev -> dig_H4 = ((uint16_t)(CalibData2[3] * 16) | ((uint16_t)CalibData2[4] & 0x0F));
	dev -> dig_H5 = ((uint16_t)CalibData2[5] * 16) | ((uint16_t)CalibData2[4] >> 4);
	dev -> dig_H6 = CalibData2[6];

	return 0; // OK
}


uint8_t BME280_ReadTemperature(BME280 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t TempData[3] = {};
	int32_t RawTemp = 0;

	status = BME280_ReadRegisters(dev, TEMP_MSB, TempData, 3);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	uint32_t DataMSB = (uint32_t)TempData[0] << 12;
	uint32_t DataLSB = (uint32_t)TempData[1] << 4;
	uint32_t DataXLSB = (uint32_t)TempData[2] >> 4;

	RawTemp = DataMSB | DataLSB | DataXLSB;

	// compensate and save to struct
	dev -> Temp_C = BME280_TemperatureCompesation(&(*dev), RawTemp);

	return 0; // OK
}


int32_t BME280_TemperatureCompesation(BME280 *dev, int32_t RawTemp){

	int32_t var1, var2, temperature;
	var1 = ((((RawTemp >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
	var2 = (((((RawTemp >> 4) - ((int32_t)dev->dig_T1)) * ((RawTemp >> 4) - ((int32_t)dev->dig_T1))) >> 12) * ((int32_t)dev->dig_T3)) >> 14;

	dev->t_fine = var1 + var2;

	temperature = (dev->t_fine * 5 + 128) >> 8;

	return temperature; // OK
}


uint8_t BME280_ReadPressure(BME280 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t PressData[3];
	int32_t RawPressure;

	status = BME280_ReadRegisters(dev, PRESS_MSB, PressData, 3);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	uint32_t DataMSB = (uint32_t)PressData[0] << 12;
	uint32_t DataLSB = (uint32_t)PressData[1] << 4;
	uint32_t DataXLSB = (uint32_t)PressData[2] >> 4;

	RawPressure = DataMSB | DataLSB | DataXLSB;

	// compesate and save to struct
	uint32_t press = BME280_PressureCompesation(&(*dev), RawPressure);
	dev -> Press_Pa = (press / 256);

	return 0; // OK
}


uint32_t BME280_PressureCompesation(BME280 *dev, int32_t RawPress){

	int64_t var1, var2, pressure;
	var1 = ((int64_t)dev->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dev->dig_P6;
	var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
	var2 = var2 + (((int64_t)dev->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->dig_P1) >> 33;

	if (var1 == 0)
	{
		return 0;
	}

	pressure = 1048576 - RawPress;
	pressure = (((pressure << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dev->dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
	var2 = (((int64_t)dev->dig_P8) * pressure) >> 19;
	pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)dev->dig_P7) << 4);
	return (uint32_t)pressure;
}


uint8_t BME280_ReadHumidity(BME280 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status = 0;
	uint8_t HumData[2];
	int32_t RawHumidity;

	status = BME280_ReadRegisters(dev, HUM_MSB, HumData, 2);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	uint32_t DataMSB = (uint32_t)HumData[0] << 8;
	uint32_t DataLSB = (uint32_t)HumData[1];

	RawHumidity = DataMSB | DataLSB;

	// compesate and save to struct
	uint32_t hum = BME280_HumidityCompesation(&(*dev), RawHumidity);
	dev -> Hum_Perc = (hum / 1024);

	return 0; // OK
}


uint32_t BME280_HumidityCompesation(BME280 *dev, int32_t RawHumidity){

	int32_t humidity;
	humidity = ((dev->t_fine) - ((int32_t)76800));
	humidity = (((((RawHumidity << 14) - (((int32_t)dev->dig_H4) << 20) - (((int32_t)dev->dig_H5) * humidity)) + ((int32_t)16384)) >> 15) * (((((((humidity * ((int32_t)dev->dig_H6)) >> 10) * (((humidity * ((int32_t)dev->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dev->dig_H2) + 8192) >> 14));
	humidity = (humidity - (((((humidity >> 15) * (humidity >> 15)) >> 7) * ((int32_t)dev->dig_H1)) >> 4));
	humidity = (humidity < 0 ? 0 : humidity);
	humidity = (humidity > 419430400 ? 419430400 : humidity);
	return (uint32_t)(humidity >> 12);
}


uint8_t BME280_ReadAllData(BME280 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status = 0;
	uint8_t Data[8] = {};
	int32_t RawTemp = 0;
	int32_t RawPressure = 0;
	int32_t RawHumidity = 0;

	status = BME280_ReadRegisters(dev, PRESS_MSB, Data, 8);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	// Save all data and combine
	uint32_t DataMSB = (uint32_t)Data[3] << 12;
	uint32_t DataLSB = (uint32_t)Data[4] << 4;
	uint32_t DataXLSB = (uint32_t)Data[5] >> 4;
	RawTemp = DataMSB | DataLSB | DataXLSB;


	DataMSB = (uint32_t)Data[0] << 12;
	DataLSB = (uint32_t)Data[1] << 4;
	DataXLSB = (uint32_t)Data[2] >> 4;
	RawPressure = DataMSB | DataLSB | DataXLSB;


	DataMSB = (uint32_t)Data[6] << 8;
	DataLSB = (uint32_t)Data[7];
	RawHumidity = DataMSB | DataLSB;

	// compesate and save to struct
	dev -> Temp_C = BME280_TemperatureCompesation(&(*dev), RawTemp);

	uint32_t press = BME280_PressureCompesation(&(*dev), RawPressure);
	dev -> Press_Pa = (press / 256);

	uint32_t hum = BME280_HumidityCompesation(&(*dev), RawHumidity);
	dev -> Hum_Perc = (hum / 1024);

	return 0; // OK
}


/* LL Drivers */
HAL_StatusTypeDef BME280_ReadRegister(BME280 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read (dev -> i2cHandle, BME280_ID, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

HAL_StatusTypeDef BME280_ReadRegisters(BME280 *dev, uint8_t reg, uint8_t *data, uint8_t lenght){
	return HAL_I2C_Mem_Read (dev -> i2cHandle, BME280_ID, reg, I2C_MEMADD_SIZE_8BIT, data, lenght, 100);
}

HAL_StatusTypeDef BME280_WriteRegister(BME280 *dev, uint8_t reg, uint8_t data){
	return HAL_I2C_Mem_Write (dev -> i2cHandle, BME280_ID, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

#endif /* BME280_C_ */
