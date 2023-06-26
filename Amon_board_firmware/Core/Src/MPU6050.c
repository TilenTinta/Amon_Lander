/*
 * MPU6050.c
 *
 *  Created on: Apr 8, 2023
 *      Author: Tinta T.
 */
#include "MPU6050.h"

/*
 * Private functions
 */
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050 *dev, uint8_t reg, uint8_t *data);						// Read one register from device

HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050 *dev, uint8_t reg, uint8_t *data, uint16_t lenght);		// Read multiple registers from device

HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050 *dev, uint8_t reg, uint8_t data);						// Write register to device

uint16_t MPU6050_ReadBytesInFIFO(MPU6050 *dev);															// Read number of bytes available to read in FIFO register


uint8_t MPU6050_ReadDeviceID(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t regData;

	status = MPU6050_ReadRegister(dev, WHO_AM_I, &regData);

	if (status != HAL_OK){
		return 1;	// NOK
	}
	else
	{
		if (regData != 0x68)
		{
			return 2; // NOK
		}
		else
		{
			return 0; // OK
		}
	}
}


uint8_t MPU6050_Reset(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;

	status = MPU6050_WriteRegister(dev, PWR_MGMT_1, 0x80); // reset device

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	HAL_Delay(10);

	status = MPU6050_WriteRegister(dev, SIGNAL_PATH_RESET, 0x07); // reset all three sensors

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	dev->GYRO_X = 0;
	dev->GYRO_Y = 0;
	dev->GYRO_Z = 0;
	dev->ACCEL_X = 0;
	dev->ACCEL_Y = 0;
	dev->ACCEL_Z = 0;
	dev->Temp_C = 0;
	dev->FT_XA = 0;
	dev->FT_YA = 0;
	dev->FT_ZA = 0;
	dev->FT_XG = 0;
	dev->FT_YG = 0;
	dev->FT_ZG = 0;

	return 0; // OK
}


uint8_t MPU6050_Init(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;

	//  Disable i2c master mode
	status = MPU6050_WriteRegister(dev, I2C_MST_CTRL, 0x00);

	if (status != HAL_OK) return 1;	// NOK

	//  PLL with X axis gyroscope reference
	status = MPU6050_WriteRegister(dev, PWR_MGMT_1, 0x01);

	if (status != HAL_OK) return 1;	// NOK

	// set low-pass filter
	status = MPU6050_WriteRegister(dev, CONFIG_MPU6050, 0x01);

	if (status != HAL_OK) return 1;	// NOK

	// set range of gyro (+-250deg/s)
	status = MPU6050_WriteRegister(dev, GYRO_CONFIG, 0x00);

	if (status != HAL_OK) return 1;	// NOK

	// set range of accel (+-8g)
	status = MPU6050_WriteRegister(dev, ACCEL_CONFIG, 0x10);

	if (status != HAL_OK) return 1;	// NOK

	// enable FIFO register
	status = MPU6050_WriteRegister(dev, USER_CTRL, 0x40);

	if (status != HAL_OK) return 1;	// NOK

	// Sample rate divider: 1kHz
	status = MPU6050_WriteRegister(dev, SMPRT_DIV, 0x01);

	if (status != HAL_OK) return 1;	// NOK

	// turn on all needed fifo enable bits to write in fifo register
	status = MPU6050_WriteRegister(dev, FIFO_EN, 0xf8);

	if (status != HAL_OK) return 1;	// NOK

	return 0; // OK
}


uint8_t MPU6050_ReadFactoryTrim(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t FTData[4];

	status = MPU6050_ReadRegisters(dev, SELF_TEST_X, FTData, 4);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	// Gyro X
	uint8_t val = (FTData[0] & 0x01F);
	if (val != 0)
	{
		dev->FT_XG = 25 * 131 * pow(1.046, (val - 1));
	}
	else
	{
		dev->FT_XG = 0;
	}

	// Gyro Y
	val = (FTData[1] & 0x01F);
	if (val != 0)
	{
		dev->FT_YG = -1 * 25 * 131 * pow(1.046, (val - 1));
	}
	else
	{
		dev->FT_YG = 0;
	}

	// Gyro Z
	val = (FTData[2] & 0x01F);
	if (val != 0)
	{
		dev->FT_ZG = 25 * 131 * pow(1.046, (val - 1));
	}
	else
	{
		dev->FT_ZG = 0;
	}

	// Accel X
	uint8_t valMSB = (FTData[0] & 0xE0) >> 5;
	uint8_t valLSB = (FTData[3] & 0x30) >> 4;
	val = (valMSB << 2) | valLSB;
	if (val != 0)
	{
		dev->FT_XA = 4096 * 0.34 * (pow(0.92, ((val - 1) / (pow(2,5) -2))) / 0.34);
	}
	else
	{
		dev->FT_XA = 0;
	}

	// Accel Y
	valMSB = (FTData[1] & 0xE0) >> 5;
	valLSB = (FTData[3] & 0x0C) >> 2;
	val = (valMSB << 2) | valLSB;
	if (val != 0)
	{
		dev->FT_YA = 4096 * 0.34 * (pow(0.92, ((val - 1) / (pow(2,5) -2))) / 0.34);
	}
	else
	{
		dev->FT_YA = 0;
	}

	// Accel Z
	valMSB = (FTData[2] & 0xE0) >> 5;
	valLSB = (FTData[3] & 0x03);
	val = (valMSB << 2) | valLSB;
	if (val != 0)
	{
		dev->FT_ZA = 4096 * 0.34 * (pow(0.92, ((val - 1) / (pow(2,5) -2))) / 0.34);
	}
	else
	{
		dev->FT_ZA = 0;
	}
	return 0; // OK
}


void MPU6050_RawToDeg(MPU6050 *dev, AMON_Drone *drone){

	/* Axis orientation on drone are: X+ points down, Z+ points out of sensor and Y+ points right if you watch drone from the board side */
	float pitch = 0;
	float roll = 0;

	/* Calculate drone pitch */
	pitch = atan(-dev->ACCEL_Z / sqrtf(pow(dev->ACCEL_Y,2) + pow(-dev->ACCEL_X,2))) * (float)(1.0f / (3.14f / 180.0f));

	/* Calculate drone Roll */
	roll = atan(-dev->ACCEL_Y / sqrtf(pow(dev->ACCEL_Z,2) + pow(-dev->ACCEL_X,2))) * (float)(1.0f / (3.14f / 180.0f));

	/* Complementary Filter */
	drone->Pitch = ALPHA * (drone->PitchOld + dev->ACCEL_Y * 0.005) + (1 - ALPHA) * pitch;
	drone->PitchOld = drone->Pitch;

	drone->Roll = ALPHA * (drone->RollOld + dev->ACCEL_Z * 0.005) + (1 - ALPHA) * roll;
	drone->RollOld = drone->Roll;
}


uint8_t MPU6050_SelfTest(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;
	HAL_StatusTypeDef status;

	/* set range of gyro (+-250deg/s) */
	status = MPU6050_WriteRegister(dev, GYRO_CONFIG, 0x00);

	if (status != HAL_OK) return 1;	// NOK

	/* set rabge of accel (+-8g) */
	status = MPU6050_WriteRegister(dev, ACCEL_CONFIG, 0x10);

	if (status != HAL_OK) return 1;	// NOK

	uint8_t GyroData[6] = {};	// Gyro data with Self-Test enabled
	uint8_t AccelData[6] = {};	// Accel data with Self-Test enabled

	uint16_t gyroOutXSTE = 0;
	uint16_t gyroOutYSTE = 0;
	uint16_t gyroOutZSTE = 0;
	uint16_t gyroOutXSTD = 0;
	uint16_t gyroOutYSTD = 0;
	uint16_t gyroOutZSTD = 0;

	uint16_t STResponseGX = 0;
	uint16_t STResponseGY = 0;
	uint16_t STResponseGZ = 0;

	float GXResoult = 0;
	float GYResoult = 0;
	float GZResoult = 0;

	uint16_t accelOutXSTE = 0;
	uint16_t accelOutYSTE = 0;
	uint16_t accelOutZSTE = 0;
	uint16_t accelOutXSTD = 0;
	uint16_t accelOutYSTD = 0;
	uint16_t accelOutZSTD = 0;

	uint16_t STResponseAX = 0;
	uint16_t STResponseAY = 0;
	uint16_t STResponseAZ = 0;

	float AXResoult = 0;
	float AYResoult = 0;
	float AZResoult = 0;

	/*** Read gyro data with self-test disabled ***/
	status = MPU6050_ReadRegisters(dev, GYRO_XOUT_H, GyroData, 6);

	if (status != HAL_OK) return 1;	// NOK

	gyroOutXSTD = ((uint16_t)GyroData[0] << 8) | GyroData[1];
	gyroOutYSTD = ((uint16_t)GyroData[2] << 8) | GyroData[3];
	gyroOutZSTD = ((uint16_t)GyroData[4] << 8) | GyroData[5];

	status = MPU6050_WriteRegister(dev, GYRO_CONFIG, 0xE0); // enable self test on all axies

	if (status != HAL_OK) return 1;	// NOK

	HAL_Delay(10);


	/* Read gyro data with self-test enabled */
	status = MPU6050_ReadRegisters(dev, GYRO_XOUT_H, GyroData, 6);

	if (status != HAL_OK) return 1;	// NOK

	gyroOutXSTE = ((uint16_t)GyroData[0] << 8) | GyroData[1];
	gyroOutYSTE = ((uint16_t)GyroData[2] << 8) | GyroData[3];
	gyroOutZSTE = ((uint16_t)GyroData[4] << 8) | GyroData[5];

	STResponseGX = gyroOutXSTE - gyroOutXSTD;
	STResponseGY = gyroOutYSTE - gyroOutYSTD;
	STResponseGZ = gyroOutZSTE - gyroOutZSTD;

	GXResoult = (float)((STResponseGX - dev->FT_XG) / dev->FT_XG); // in percent
	if (GXResoult > 14 || GXResoult < -14 ) return 1;

	GYResoult = (float)((STResponseGY - dev->FT_YG) / dev->FT_YG); // in percent
	if (GYResoult > 14 || GYResoult < -14 ) return 1;

	GZResoult = (float)((STResponseGZ - dev->FT_ZG) / dev->FT_ZG); // in percent
	if (GZResoult > 14 || GZResoult < -14 ) return 1;


	status = MPU6050_WriteRegister(dev, GYRO_CONFIG, 0x00); // enable self test on all axies

	if (status != HAL_OK) return 1;	// NOK


	/*** Read accel data with self-test disabled ***/
	status = MPU6050_ReadRegisters(dev, ACCEL_XOUT_H, AccelData, 6);

	if (status != HAL_OK) return 1;	// NOK

	accelOutXSTD = ((uint16_t)AccelData[0] << 8) | AccelData[1];
	accelOutYSTD = ((uint16_t)AccelData[2] << 8) | AccelData[3];
	accelOutZSTD = ((uint16_t)AccelData[4] << 8) | AccelData[5];

	status = MPU6050_WriteRegister(dev, ACCEL_CONFIG, 0xE0); // enable self test on all axies

	if (status != HAL_OK) return 1;	// NOK

	HAL_Delay(10);


	/* Read gyro data with self-test enabled */
	status = MPU6050_ReadRegisters(dev, ACCEL_XOUT_H, AccelData, 6);

	if (status != HAL_OK) return 1;	// NOK

	accelOutXSTE = ((uint16_t)AccelData[0] << 8) | AccelData[1];
	accelOutYSTE = ((uint16_t)AccelData[2] << 8) | AccelData[3];
	accelOutZSTE = ((uint16_t)AccelData[4] << 8) | AccelData[5];

	STResponseAX = accelOutXSTE - accelOutXSTD;
	STResponseAY = accelOutYSTE - accelOutYSTD;
	STResponseAZ = accelOutZSTE - accelOutZSTD;

	AXResoult = (float)((STResponseAX - dev->FT_XA) / dev->FT_XA); // in percent
	if (AXResoult > 15 || AXResoult < -14 ) return 1; // 14

	AYResoult = (float)((STResponseAY - dev->FT_YA) / dev->FT_YA); // in percent
	if (AYResoult > 14 || AYResoult < -14 ) return 1;

	AZResoult = (float)((STResponseAZ - dev->FT_ZA) / dev->FT_ZA); // in percent
	if (AZResoult > 14 || AZResoult < -14 ) return 1;

	status = MPU6050_WriteRegister(dev, ACCEL_CONFIG, 0x10); // enable self test on all axies

	if (status != HAL_OK) return 1;	// NOK


	return 0; // OK
}


uint8_t MPU6050_ReadAllDirect(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t Data[14] = {0};

	status = MPU6050_ReadRegisters(dev, ACCEL_XOUT_H, Data, 14);

	if (status != HAL_OK) return 1;	// NOK

	// Acceleration X
	uint16_t DataMSB = (uint16_t)Data[0] << 8;
	uint16_t DataLSB = (uint16_t)Data[1];

	int16_t RawAccelX = DataMSB | DataLSB;

	dev->ACCEL_X= (float)RawAccelX / 4096.0f + (float)(X_ACCEL_OFFSET);

	// Acceleration Y
	DataMSB = (uint16_t)Data[2] << 8;
	DataLSB = (uint16_t)Data[3];

	int16_t RawAccelY = DataMSB | DataLSB;

	dev->ACCEL_Y= (float)RawAccelY / 4096.0f + (float)(Y_ACCEL_OFFSET);

	// Acceleration Z
	DataMSB = (uint16_t)Data[4] << 8;
	DataLSB = (uint16_t)Data[5];

	int16_t RawAccelZ = DataMSB | DataLSB;

	dev->ACCEL_Z= (float)RawAccelZ / 4096.0f + (float)(Z_ACCEL_OFFSET);

	// Temperature
	DataMSB = (uint16_t)Data[6] << 8;
	DataLSB = (uint16_t)Data[7];

	uint16_t RawTemp = DataMSB | DataLSB;

	dev->Temp_C = (int16_t)((((float)RawTemp) / 340 + 36.53f) * 10);

	// Gyro X
	DataMSB = (uint16_t)Data[8] << 8;
	DataLSB = (uint16_t)Data[9];

	int16_t RawGyroX = DataMSB | DataLSB;

	dev->GYRO_X = (float)RawGyroX / 131.0f;

	// Gyro Y
	DataMSB = (uint16_t)Data[10] << 8;
	DataLSB = (uint16_t)Data[11];

	int16_t RawGyroY = DataMSB | DataLSB;

	dev->GYRO_Y = (float)RawGyroY / 131.0f;

	// Gyro Z
	DataMSB = (uint16_t)Data[12] << 8;
	DataLSB = (uint16_t)Data[13];

	int16_t RawGyroZ = DataMSB | DataLSB;

	dev->GYRO_Z = (float)RawGyroZ / 131.0f;

	return 0; // OK
}


uint8_t MPU6050_ReadTemperatureDirect(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t TempData[2] = {};
	uint16_t RawTemp;

	status = MPU6050_ReadRegisters(dev, TEMP_OUT_H, TempData, 2);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	uint16_t DataMSB = TempData[0] << 8;
	uint16_t DataLSB = TempData[1];

	RawTemp = DataMSB | DataLSB;

	dev->Temp_C = (int16_t)((((float)RawTemp) / 340 + 36.53f) * 10);

	return 0; // OK
}


uint8_t MPU6050_ReadGyroXDirect(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t GyroXData[2] = {};
	int16_t RawGyroX;

	status = MPU6050_ReadRegisters(dev, GYRO_XOUT_H, GyroXData, 2);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	uint16_t DataMSB = (uint16_t)GyroXData[0] << 8;
	uint16_t DataLSB = (uint16_t)GyroXData[1];

	RawGyroX = DataMSB | DataLSB;

	dev->GYRO_X = (float)RawGyroX / 131.0f;

	return 0; // OK
}


uint8_t MPU6050_ReadGyroYDirect(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t GyroYData[2] = {};
	int16_t RawGyroY;

	status = MPU6050_ReadRegisters(dev, GYRO_YOUT_H, GyroYData, 2);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	uint16_t DataMSB = (uint16_t)GyroYData[0] << 8;
	uint16_t DataLSB = (uint16_t)GyroYData[1];

	RawGyroY = DataMSB | DataLSB;

	dev->GYRO_Y = (float)RawGyroY / 131.0f;

	return 0; // OK
}


uint8_t MPU6050_ReadGyroZDirect(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t GyroZData[2] = {};
	int16_t RawGyroZ;

	status = MPU6050_ReadRegisters(dev, GYRO_ZOUT_H, GyroZData, 2);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	uint16_t DataMSB = (uint16_t)GyroZData[0] << 8;
	uint16_t DataLSB = (uint16_t)GyroZData[1];

	RawGyroZ = DataMSB | DataLSB;

	dev->GYRO_Z = (float)RawGyroZ / 131.0f;

	return 0; // OK
}


uint8_t MPU6050_ReadAccelXDirect(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t AccelXData[2] = {};
	int16_t RawAccelX;

	status = MPU6050_ReadRegisters(dev, ACCEL_XOUT_H, AccelXData, 2);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	uint16_t DataMSB = (uint16_t)AccelXData[0] << 8;
	uint16_t DataLSB = (uint16_t)AccelXData[1];

	RawAccelX = DataMSB | DataLSB;

	dev->ACCEL_X= (float)RawAccelX / 4096.0f + (X_ACCEL_OFFSET);

	return 0; // OK
}


uint8_t MPU6050_ReadAccelYDirect(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t AccelYData[2] = {};
	int16_t RawAccelY;

	status = MPU6050_ReadRegisters(dev, ACCEL_YOUT_H, AccelYData, 2);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	uint16_t DataMSB = (uint16_t)AccelYData[0] << 8;
	uint16_t DataLSB = (uint16_t)AccelYData[1];

	RawAccelY = DataMSB | DataLSB;

	dev->ACCEL_Y= (float)RawAccelY / 4096.0f + (Y_ACCEL_OFFSET);

	return 0; // OK
}


uint8_t MPU6050_ReadAccelZDirect(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status;
	uint8_t AccelZData[2] = {};
	int16_t RawAccelZ;

	status = MPU6050_ReadRegisters(dev, ACCEL_ZOUT_H, AccelZData, 2);

	if (status != HAL_OK)
	{
		return 1;	// NOK
	}

	uint16_t DataMSB = (uint16_t)AccelZData[0] << 8;
	uint16_t DataLSB = (uint16_t)AccelZData[1];

	RawAccelZ = DataMSB | DataLSB;

	dev->ACCEL_Z= (float)RawAccelZ / 4096.0f + (Z_ACCEL_OFFSET);

	return 0; // OK
}


uint8_t MPU6050_ReadFIFO(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){ // NOT WORKING YET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	dev -> i2cHandle = i2cHandle;

	HAL_StatusTypeDef status = 0;
	uint16_t lenght = 0;

	// clear FIFO
	status = MPU6050_WriteRegister(dev, USER_CTRL, 0x44);

	if (status != HAL_OK) return 1;	// NOK

	// turn on all needed fifo enable bits to write in fifo register
	status = MPU6050_WriteRegister(dev, FIFO_EN, 0xf8);

	if (status != HAL_OK) return 1;	// NOK


	uint8_t test = 0;
	status = MPU6050_ReadRegister(dev, USER_CTRL, &test);

	if (status != HAL_OK) return 1;	// NOK

	// read amount of bytes copied to FIFO register
	lenght = MPU6050_ReadBytesInFIFO(&(*dev));

	while(lenght != 14)
	{
		lenght = MPU6050_ReadBytesInFIFO(&(*dev));
	}

	// read data from FIFO register
	uint8_t FIFOData[lenght];
	uint16_t ReadCnt = 0;

	while(ReadCnt != lenght)
	{
		status = MPU6050_ReadRegister(dev, FIFO_R_W, &FIFOData[ReadCnt]);
		ReadCnt++;

		if (status != HAL_OK)
		{
			return 1;	// NOK
		}
	}

	uint16_t RawTemp;
	uint16_t DataMSB = FIFOData[0] << 8;
	uint16_t DataLSB = FIFOData[1];

	RawTemp = DataMSB | DataLSB;

	dev->Temp_C = (int16_t)((((float)RawTemp) / 340 + 36.53f) * 10);

	return 0; // OK
}


uint16_t MPU6050_ReadBytesInFIFO(MPU6050 *dev){

	HAL_StatusTypeDef status;
	uint8_t Data[2] = {0};
	uint16_t DataInBuff = 0;


	status = MPU6050_ReadRegisters(dev, FIFO_COUNT_H, Data, (uint16_t)2);

	if (status != HAL_OK)
	{
		return 0;	// NOK
	}

	uint16_t DataMSB = (uint16_t)Data[1] << 8;
	uint16_t DataLSB = (uint16_t)Data[0];

	DataInBuff = DataMSB | DataLSB;

	return DataInBuff; // OK
}


//* LL functions *//
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050 *dev, uint8_t reg, uint8_t *data){
	return HAL_I2C_Mem_Read (dev -> i2cHandle, MPU6050_ID, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
};

HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050 *dev, uint8_t reg, uint8_t *data, uint16_t lenght){
	return HAL_I2C_Mem_Read (dev -> i2cHandle, MPU6050_ID, reg, I2C_MEMADD_SIZE_8BIT, data, lenght, 100);
};

HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050 *dev, uint8_t reg, uint8_t data){
	return HAL_I2C_Mem_Write (dev -> i2cHandle, MPU6050_ID, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
};
