
#ifndef _API_H_
#define _API_H_

#include "vl53l1_platform.h"
#include "stm32f4xx_hal.h"


#define VL53L1X_IMPLEMENTATION_VER_MAJOR       3
#define VL53L1X_IMPLEMENTATION_VER_MINOR       5
#define VL53L1X_IMPLEMENTATION_VER_SUB         2
#define VL53L1X_IMPLEMENTATION_VER_REVISION  0000

typedef int8_t VL53L1X_ERROR;

#define SOFT_RESET											0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS					0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND        0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 		0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 	0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 	0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM					0x001E
#define MM_CONFIG__INNER_OFFSET_MM							0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 							0x0022
#define GPIO_HV_MUX__CTRL									0x0030
#define GPIO__TIO_HV_STATUS       							0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO 						0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP     				0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI   				0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A        				0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B						0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI  					0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO  					0x0062
#define RANGE_CONFIG__SIGMA_THRESH 							0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS			0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH      				0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD				0x006C
#define SYSTEM__THRESH_HIGH 								0x0072
#define SYSTEM__THRESH_LOW 									0x0074
#define SD_CONFIG__WOI_SD0                  				0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0        				0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD					0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE		0x0080
#define SYSTEM__SEQUENCE_CONFIG								0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD 				0x0082
#define SYSTEM__INTERRUPT_CLEAR       						0x0086
#define SYSTEM__MODE_START                 					0x0087
#define VL53L1_RESULT__RANGE_STATUS							0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0		0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD					0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0				0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 	0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL					0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS                      0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID                     0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD				0x013E


typedef struct {
	uint8_t      major;    /*!< major number */
	uint8_t      minor;    /*!< minor number */
	uint8_t      build;    /*!< build number */
	uint32_t     revision; /*!< revision number */
} VL53L1X_Version_t;


typedef struct {
	uint8_t Status;		/*!< ResultStatus */
	uint16_t Distance;	/*!< ResultDistance */
	uint16_t Ambient;	/*!< ResultAmbient */
	uint16_t SigPerSPAD;/*!< ResultSignalPerSPAD */
	uint16_t NumSPADs;	/*!< ResultNumSPADs */
} VL53L1X_Result_t;

/**
 * This function returns the SW driver version
 */
VL53L1X_ERROR VL53L1X_GetSWVersion(VL53L1X_Version_t *pVersion);

/**
 * This function sets the sensor I2C address used in case multiple devices application, default address 0x52
 */
VL53L1X_ERROR VL53L1X_SetI2CAddress(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint8_t new_address);

/**
 * This function loads the 135 bytes default values to initialize the sensor.
 * 0:success, != 0:failed
 */
VL53L1X_ERROR VL53L1X_SensorInit(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle);

/**
 * This function clears the interrupt, to be called after a ranging data reading
 * to arm the interrupt for the next data ready event.
 */
VL53L1X_ERROR VL53L1X_ClearInterrupt(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle);

/**
 * This function programs the interrupt polarity\n
 * 1=active high (default), 0=active low
 */
VL53L1X_ERROR VL53L1X_SetInterruptPolarity(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint8_t IntPol);

/**
 * This function returns the current interrupt polarity\n
 * 1=active high (default), 0=active low
 */
VL53L1X_ERROR VL53L1X_GetInterruptPolarity(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint8_t *pIntPol);

/**
 * This function starts the ranging distance operation\n
 * The ranging operation is continuous. The clear interrupt has to be done after each get data to allow the interrupt to raise when the next data is ready\n
 * 1=active high (default), 0=active low, use SetInterruptPolarity() to change the interrupt polarity if required.
 */
VL53L1X_ERROR VL53L1X_StartRanging(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle);

/**
 * This function stops the ranging.
 */
VL53L1X_ERROR VL53L1X_StopRanging(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle);

/**
 * This function checks if the new ranging data is available by polling the dedicated register.
 * isDataReady==0 -> not ready; isDataReady==1 -> ready
 */
VL53L1X_ERROR VL53L1X_CheckForDataReady(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint8_t *isDataReady);

/**
 * This function programs the timing budget in ms.
 * Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
 */
VL53L1X_ERROR VL53L1X_SetTimingBudgetInMs(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t TimingBudgetInMs);

/**
 * This function returns the current timing budget in ms.
 */
VL53L1X_ERROR VL53L1X_GetTimingBudgetInMs(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *pTimingBudgetInMs);

/**
 * This function programs the distance mode (1=short, 2=long(default)).
 * Short mode max distance is limited to 1.3 m but better ambient immunity.\n
 * Long mode can range up to 4 m in the dark with 200 ms timing budget.
 */
VL53L1X_ERROR VL53L1X_SetDistanceMode(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t DistanceMode);

/**
 * This function returns the current distance mode (1=short, 2=long).
 */
VL53L1X_ERROR VL53L1X_GetDistanceMode(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *pDistanceMode);

/**
 * This function programs the Intermeasurement period in ms\n
 * Intermeasurement period must be >/= timing budget. This condition is not checked by the API,
 * the customer has the duty to check the condition. Default = 100 ms
 */
VL53L1X_ERROR VL53L1X_SetInterMeasurementInMs(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint32_t InterMeasurementInMs);

/**
 * This function returns the Intermeasurement period in ms.
 */
VL53L1X_ERROR VL53L1X_GetInterMeasurementInMs(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t * pIM);

/**
 * This function returns the boot state of the device (1:booted, 0:not booted)
 */
VL53L1X_ERROR VL53L1X_BootState(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint8_t *state);

/**
 * This function returns the sensor id, sensor Id must be 0xEEAC
 */
VL53L1X_ERROR VL53L1X_GetSensorId(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *id);

/**
 * This function returns the distance measured by the sensor in mm
 */
VL53L1X_ERROR VL53L1X_GetDistance(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *distance);

/**
 * This function returns the returned signal per SPAD in kcps/SPAD.
 * With kcps stands for Kilo Count Per Second
 */
VL53L1X_ERROR VL53L1X_GetSignalPerSpad(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *signalPerSp);

/**
 * This function returns the ambient per SPAD in kcps/SPAD
 */
VL53L1X_ERROR VL53L1X_GetAmbientPerSpad(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *amb);

/**
 * This function returns the returned signal in kcps.
 */
VL53L1X_ERROR VL53L1X_GetSignalRate(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *signalRate);

/**
 * This function returns the current number of enabled SPADs
 */
VL53L1X_ERROR VL53L1X_GetSpadNb(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *spNb);

/**
 * This function returns the ambient rate in kcps
 */
VL53L1X_ERROR VL53L1X_GetAmbientRate(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *ambRate);

/**
 * This function returns the ranging status error \n
 * (0:no error, 1:sigma failed, 2:signal failed, ..., 7:wrap-around)
 */
VL53L1X_ERROR VL53L1X_GetRangeStatus(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint8_t *rangeStatus);

/**
 * This function returns measurements and the range status in a single read access
 */
VL53L1X_ERROR VL53L1X_GetResult(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, VL53L1X_Result_t *pResult);

/**
 * This function programs the offset correction in mm
 * OffsetValue:the offset correction value to program in mm
 */
VL53L1X_ERROR VL53L1X_SetOffset(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, int16_t OffsetValue);

/**
 * This function returns the programmed offset correction value in mm
 */
VL53L1X_ERROR VL53L1X_GetOffset(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, int16_t *Offset);

/**
 * This function programs the xtalk correction value in cps (Count Per Second).\n
 * This is the number of photons reflected back from the cover glass in cps.
 */
VL53L1X_ERROR VL53L1X_SetXtalk(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t XtalkValue);

/**
 * This function returns the current programmed xtalk correction value in cps
 */
VL53L1X_ERROR VL53L1X_GetXtalk(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *Xtalk);

/**
 * This function programs the threshold detection mode\n
 * Example:\n
 * VL53L1X_SetDistanceThreshold(dev,100,300,0,1): Below 100 \n
 * VL53L1X_SetDistanceThreshold(dev,100,300,1,1): Above 300 \n
 * VL53L1X_SetDistanceThreshold(dev,100,300,2,1): Out of window \n
 * VL53L1X_SetDistanceThreshold(dev,100,300,3,1): In window \n
 *  dev : device address
 *  ThreshLow(in mm) : the threshold under which one the device raises an interrupt if Window = 0
 *  ThreshHigh(in mm) :  the threshold above which one the device raises an interrupt if Window = 1
 *  Window detection mode : 0=below, 1=above, 2=out, 3=in
 *  IntOnNoTarget = 0 (No longer used - just use 0)
 */
VL53L1X_ERROR VL53L1X_SetDistanceThreshold(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t ThreshLow, uint16_t ThreshHigh, uint8_t Window, uint8_t IntOnNoTarget);

/**
 * This function returns the window detection mode (0=below; 1=above; 2=out; 3=in)
 */
VL53L1X_ERROR VL53L1X_GetDistanceThresholdWindow(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *window);

/**
 * This function returns the low threshold in mm
 */
VL53L1X_ERROR VL53L1X_GetDistanceThresholdLow(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *low);

/**
 * This function returns the high threshold in mm
 */
VL53L1X_ERROR VL53L1X_GetDistanceThresholdHigh(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *high);

/**
 * This function programs the ROI (Region of Interest)\n
 * The ROI position is centered, only the ROI size can be reprogrammed.\n
 * The smallest acceptable ROI size = 4\n
 * X:ROI Width; Y=ROI Height
 */
VL53L1X_ERROR VL53L1X_SetROI(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t X, uint16_t Y);

/**
 * This function returns width X and height Y
 */
VL53L1X_ERROR VL53L1X_GetROI_XY(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *ROI_X, uint16_t *ROI_Y);

/**
 * This function programs the new user ROI center, please to be aware that there is no check in this function.
 * if the ROI center vs ROI size is out of border the ranging function return error #13
 */
VL53L1X_ERROR VL53L1X_SetROICenter(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint8_t ROICenter);

/**
 * This function returns the current user ROI center
 */
VL53L1X_ERROR VL53L1X_GetROICenter(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint8_t *ROICenter);

/**
 * This function programs a new signal threshold in kcps (default=1024 kcps\n
 */
VL53L1X_ERROR VL53L1X_SetSignalThreshold(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t signal);

/**
 * This function returns the current signal threshold in kcps
 */
VL53L1X_ERROR VL53L1X_GetSignalThreshold(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *signal);

/**
 * @brief This function programs a new sigma threshold in mm (default=15 mm)
 */
VL53L1X_ERROR VL53L1X_SetSigmaThreshold(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t sigma);

/*
 * This function returns the current sigma threshold in mm
 */
VL53L1X_ERROR VL53L1X_GetSigmaThreshold(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle, uint16_t *signal);

/*
 * This function performs the temperature calibration.
 * It is recommended to call this function any time the temperature might have changed by more than 8 deg C
 * without sensor ranging activity for an extended period.
 */
VL53L1X_ERROR VL53L1X_StartTemperatureUpdate(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle);

/*
 * Read sensor model ID
 */
VL53L1X_ERROR VL53L1X_ReadID(VL53L1_DEV *dev, I2C_HandleTypeDef *i2cHandle);

#endif
