
#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_


int8_t VL53L1X_CalibrateOffset(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t TargetDistInMm, int16_t *offset);

int8_t VL53L1X_CalibrateXtalk(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t TargetDistInMm, uint16_t *xtalk);

#endif
