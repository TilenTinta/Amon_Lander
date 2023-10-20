/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Functions init */
uint16_t ADC_Read_Main_Battery();
uint16_t ADC_Read_EDF_Battery();
uint8_t TVCServoEnable();
uint8_t TVCServoDisable();
uint8_t EDFEnable();
uint8_t EDFDisable();
void StatusLED(uint8_t Status);

/* Variables */
uint8_t DroneStatusLocal = 0;
uint8_t DroneStatusOld = 20;
uint8_t StartupInit = 0;
uint8_t InitError = 0;

uint8_t LED_blink_cnt_ON;
uint8_t LED_blink_cnt_double;
uint8_t LED_blink_cnt_OFF;

uint8_t TVCServoEnableFlag = 0;
uint8_t EDFEnableFlag = 0;

uint8_t RF_IRQ1_EN = 0;			// flag to enable irq code
uint8_t RF_IRQ2_EN = 0;			// flag to enable irq code

float PitchDef[20] = {0};		// Default values for calculating angle before flight
float RollDef[20] = {0};
uint8_t MeassCnt = 0;			// counting measurements

// Gyroscope calibration
uint8_t GyroCalibTrig = 0;
uint16_t RGB_Red = 0;
uint16_t RGB_Green = 300;
uint16_t RGB_Blue = 600;
uint16_t RGB_RedMax = 0;
uint16_t RGB_GreenMax = 0;
uint16_t RGB_BlueMax = 0;

// Battery data
uint8_t ADC_DMA_DataRdy = 0;
uint32_t ADC_BAT_Val[2] = {};

// GPS data
uint8_t USART4_GPSRX[426] = {"\0"};
uint8_t NewGPSData = 0;

// Timers IRQ enable flag
uint8_t Reg1HzLoopEN = 0;
uint8_t Reg50HzLoopEN = 0;
uint8_t Reg200HzLoopEN = 0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  MX_FATFS_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* STRUCTS */
  MPU6050 mpu6050; // Temp, Press, Hum
  BME280 bme280; // Gyro

  AMON_Drone AmonDrone; // All drone data

  // Decoded GPS data for each type of packet
  GPS_GGA gps_gga;
  GPS_GLL gps_gll;
  GPS_GSA gps_gsa;
  GPS_GSV gps_gsv;
  GPS_RMC gps_rmc;
  GPS_VTG gps_vtg;

  VL53L1_DEV vl53l1Dev; // Lidar
  //VL53L1X_Version_t vl53l1xVersion_t; // Lidar - unused
  //VL53L1X_Result_t vl53l1xResult_t; // Lidar - unused


  /* Variables */


  // Init
  AmonDrone.DroneStatus = 0;
  AmonDrone.PitchOld = 0;
  AmonDrone.RollOld = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Changing values when changing status
	  if (AmonDrone.DroneStatus != DroneStatusOld && CALIBRATION == 0)
	  {
		  DroneStatusOld = AmonDrone.DroneStatus;
		  DroneStatusLocal = AmonDrone.DroneStatus;
		  StatusLED(AmonDrone.DroneStatus);
	  }

	  // RGB cycling when calibrating
	  if (CALIBRATION == 1)
	  {
		  AmonDrone.DroneStatus = STATUS_GYRO_CALIB;
		  DroneStatusLocal = AmonDrone.DroneStatus;
	  }




	  /****************************
	   * INIT  SEQUENCE
	   ****************************/

	  if (StartupInit == STATUS_STARTUP)
	  {

		//AmonDrone.DroneStatus = STATUS_STARTUP;
		uint8_t status = 0;

		/* Aligne all motors */
		TVCServoEnable();
		DegresToCCR(90.0 + SERVO_XN_OFFSET, SERVO_XN);
		DegresToCCR(90.0 + SERVO_XP_OFFSET, SERVO_XP);
		DegresToCCR(90.0 + SERVO_YN_OFFSET, SERVO_YN);
		DegresToCCR(90.0 + SERVO_YP_OFFSET, SERVO_YP);
		HAL_Delay(500); // wait on motors to stop mooving

		DegresToCCR(80.0 + SERVO_XN_OFFSET, SERVO_XN);
		DegresToCCR(80.0 + SERVO_XP_OFFSET, SERVO_XP);
		DegresToCCR(80.0 + SERVO_YN_OFFSET, SERVO_YN);
		DegresToCCR(80.0 + SERVO_YP_OFFSET, SERVO_YP);
		HAL_Delay(500); // wait on motors to stop mooving

		DegresToCCR(90.0 + SERVO_XN_OFFSET, SERVO_XN);
		DegresToCCR(90.0 + SERVO_XP_OFFSET, SERVO_XP);
		DegresToCCR(90.0 + SERVO_YN_OFFSET, SERVO_YN);
		DegresToCCR(90.0 + SERVO_YP_OFFSET, SERVO_YP);
		HAL_Delay(500); // wait on motors to stop mooving

		DegresToCCR(100.0 + SERVO_XN_OFFSET, SERVO_XN);
		DegresToCCR(100.0 + SERVO_XP_OFFSET, SERVO_XP);
		DegresToCCR(100.0 + SERVO_YN_OFFSET, SERVO_YN);
		DegresToCCR(100.0 + SERVO_YP_OFFSET, SERVO_YP);
		HAL_Delay(500); // wait on motors to stop mooving

		DegresToCCR(90.0 + SERVO_XN_OFFSET, SERVO_XN);
		DegresToCCR(90.0 + SERVO_XP_OFFSET, SERVO_XP);
		DegresToCCR(90.0 + SERVO_YN_OFFSET, SERVO_YN);
		DegresToCCR(90.0 + SERVO_YP_OFFSET, SERVO_YP);
		HAL_Delay(500); // wait on motors to stop mooving

		// Start timers for sensors and LEDs
		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_Brd_GPIO_Port, LED_Brd_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // RGB (50Hz)
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // RGB (50Hz)
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // RGB (50Hz)

		/* Read both batteries and save in drone data struct */
		HAL_ADC_Start_DMA(&hadc1, ADC_BAT_Val, 2); 		// Start ADC DMA (read analog value)

		while(ADC_DMA_DataRdy == 0){
			// white...
		}

		AmonDrone.MainBatVoltage = ADC_Read_Main_Battery();
		AmonDrone.EDFBatVoltage = ADC_Read_EDF_Battery();
		ADC_DMA_DataRdy = 0;

		if (AmonDrone.MainBatVoltage < 1000) status++; // check board battery voltage (more than XV)
		//if (AmonDrone.EDFBatVoltage < 2000) status++; // check board battery voltage (more than XV)

		HAL_Delay(500);

		/* Reset all devices */
		status += BME280_Reset(&bme280, &hi2c3);
		status += MPU6050_Reset(&mpu6050, &hi2c3);

		HAL_Delay(500); // delay sensors config to complete power on

		/* BME280 */
		status += BME280_ReadDeviceID(&bme280, &hi2c3);
		status += BME280_ReadCalibData(&bme280, &hi2c3);
		status += BME280_Init(&bme280, &hi2c3);

		/* MPU6050 */
		status += MPU6050_ReadDeviceID(&mpu6050, &hi2c3);
		status += MPU6050_ReadFactoryTrim(&mpu6050, &hi2c3);
		status += MPU6050_Init(&mpu6050, &hi2c3);
		status += MPU6050_ReadFactoryTrim(&mpu6050, &hi2c3);
		status += MPU6050_ReadAllDirect(&mpu6050, &hi2c3);
		status += MPU6050_SelfTest(&mpu6050, &hi2c3);

		/* vl53l1x */
		uint8_t bootOK = 0;
		status += VL53L1X_ReadID(&vl53l1Dev, &hi2c3);
		while (bootOK == 0)
		{
			status += VL53L1X_BootState(&vl53l1Dev, &hi2c3, &bootOK);
		}
		status += VL53L1X_SensorInit(&vl53l1Dev, &hi2c3);
		status += VL53L1X_SetTimingBudgetInMs(&vl53l1Dev, &hi2c3, 200); // 140ms is min for 4m distance
		status += VL53L1X_SetOffset(&vl53l1Dev, &hi2c3, -130); // Set height from ground to get zero
		VL53L1X_StartRanging(&vl53l1Dev, &hi2c3);


		HAL_TIM_Base_Start_IT(&htim4); // TVC LOOP, leg leds (50Hz)
		HAL_TIM_Base_Start_IT(&htim5); // Complementary Filter


		if (CALIBRATION == 0) HAL_UART_Receive_DMA(&huart4, USART4_GPSRX, 426); // Do not enable UART DMA if calibrating


		if (bme280.dig_T1 == 0 || bme280.dig_T2 == 0) // for WTF error
		{
			status++;
		}

		InitError = status;

		// Check if all init functionsa are OK
		if (InitError == 0) // OK
		{
			StartupInit = 1;
			AmonDrone.DroneStatus = STATUS_IDLE_NC;
		}
		else
		{
			StartupInit = 1;
			AmonDrone.DroneStatus = STATUS_ERROR; // ERROR
			HAL_GPIO_WritePin(LED_Brd_GPIO_Port, LED_Brd_Pin, GPIO_PIN_SET);
		}

	  }





	  /****************************
	   * SEQUENCE IDLE, ARM, FLY...
	   ****************************/

	  if (StartupInit != STATUS_STARTUP && AmonDrone.DroneStatus != STATUS_ERROR && CALIBRATION == 0)
	  {

		  /*** CURRENT DRONE STATUS ***/
		  switch(AmonDrone.DroneStatus){



		  /*/ Drone ideling but no connection with link /*/
		  case STATUS_IDLE_NC:

			  if (TVCServoEnableFlag == 1) TVCServoDisable();	// Disable TVC servos
			  if (EDFEnableFlag == 1) EDFDisable();				// Disable EDF


			  if (Reg200HzLoopEN == 1) // Timer 4
			  {
				  MPU6050_ReadAllDirect(&mpu6050, &hi2c3);	// read data from gyro
				  MPU6050_RawToDeg(&mpu6050, &AmonDrone); 	// calculate data to pitch and roll (and yaw)
				  Reg200HzLoopEN = 0;
			  }


			  if (Reg50HzLoopEN == 1)	// Timer 5
			  {
				  uint8_t dataRdy = 0;
				  while(dataRdy == 0)
				  {
					  VL53L1X_CheckForDataReady(&vl53l1Dev, &hi2c3, &dataRdy);
				  }
				  dataRdy = 0;
				  VL53L1X_GetDistance(&vl53l1Dev, &hi2c3, &AmonDrone.Height);
				  VL53L1X_ClearInterrupt(&vl53l1Dev, &hi2c3);
				  // Read other sensors
				  Reg50HzLoopEN = 0;
			  }


			  if (Reg1HzLoopEN == 1)	// Timer 6
			  {

				  Reg1HzLoopEN = 0;
			  }


			  if (NewGPSData == 1 && USE_GPS == 1){	// new gps data available -> decode
				  GPS_Decode_GGA(&USART4_GPSRX, &gps_gga, &AmonDrone);

				  NewGPSData = 0;
			  }

			  break;

		  /*/ Drone ideling and is connected with link /*/
		  case STATUS_IDLE_CN:

			  if (TVCServoEnableFlag == 1) TVCServoDisable();	// Disable TVC servos
			  if (EDFEnableFlag == 1) EDFDisable();				// Disable EDF


			  if (Reg200HzLoopEN == 1) // Timer 4
			  {
			  	  MPU6050_ReadAllDirect(&mpu6050, &hi2c3);
			  	  MPU6050_RawToDeg(&mpu6050, &AmonDrone); // calculate data to pitch and roll (and yaw)
			  	  Reg200HzLoopEN = 0;
			  }


			  if (Reg50HzLoopEN == 1)	// Timer 5
			  {
 				  uint8_t dataRdy = 0;
  				  while(dataRdy == 0)
  				  {
  					  VL53L1X_CheckForDataReady(&vl53l1Dev, &hi2c3, &dataRdy);
			  	  }
			  	  dataRdy = 0;
			  	  VL53L1X_GetDistance(&vl53l1Dev, &hi2c3, &AmonDrone.Height);
			  	  VL53L1X_ClearInterrupt(&vl53l1Dev, &hi2c3);
			  	// Read other sensors...
			  	  Reg50HzLoopEN = 0;
			  }


			  if (Reg1HzLoopEN == 1)	// Timer 6
			  {

				  Reg1HzLoopEN = 0;
			  }


			  if (NewGPSData == 1 && USE_GPS == 1){	// new gps data available -> decode


				  NewGPSData = 0;
			  }

			  break;

		  /*/ Drone is armed to take off /*/
		  case STATUS_ARM:

			  if (TVCServoEnableFlag == 0) TVCServoEnable();
			  if (EDFEnableFlag == 0) EDFEnable();

			  if (Reg200HzLoopEN == 1) // Timer 4
			  {
			  	  MPU6050_ReadAllDirect(&mpu6050, &hi2c3);
			  	  MPU6050_RawToDeg(&mpu6050, &AmonDrone); // calculate data to pitch and roll (and yaw)
			  	  Reg200HzLoopEN = 0;
			  }

			  if (Reg50HzLoopEN == 1)	// Timer 5
			  {

				  // Moving average for pitch and roll
				  if (MeassCnt == 20)
				  {
					  float PitchSum = 0;
					  float RollSum = 0;

					  for (int i = 0; i < 20; i++)
					  {
						  PitchSum += PitchDef[i];
						  RollSum += RollDef[i];
					  }

					  AmonDrone.PitchMean = PitchSum / 20;
					  AmonDrone.RollMean = RollSum / 20;

					  MeassCnt = 0; // reset if not used for new data
				  }

				  if (MeassCnt < 20)
				  {
					  PitchDef[MeassCnt] = AmonDrone.Pitch;
					  RollDef[MeassCnt] = AmonDrone.Roll;
					  MeassCnt++;
				  }


				  uint8_t dataRdy = 0;
				  while(dataRdy == 0)
				  {
					  VL53L1X_CheckForDataReady(&vl53l1Dev, &hi2c3, &dataRdy);
				  }
				  dataRdy = 0;
				  VL53L1X_GetDistance(&vl53l1Dev, &hi2c3, &AmonDrone.Height);
				  VL53L1X_ClearInterrupt(&vl53l1Dev, &hi2c3);
				  // Read other sensors...

				  Reg50HzLoopEN = 0;
			  }


			  if (Reg1HzLoopEN == 1)	// Timer 6
			  {

				  Reg1HzLoopEN = 0;
			  }


			  if (NewGPSData == 1 && USE_GPS == 1){	// new gps data available -> decode


				  NewGPSData = 0;
			  }

			  break;

		  /*/ Flying /*/
		  case STATUS_FLY:


			  DegresToCCR(90.0f + AmonDrone.Pitch + (AmonDrone.PitchMean) + SERVO_XN_OFFSET, SERVO_XN);
			  DegresToCCR(90.0f - AmonDrone.Pitch + (AmonDrone.PitchMean) + SERVO_XP_OFFSET, SERVO_XP);
			  DegresToCCR(90.0f - AmonDrone.Roll + (AmonDrone.RollMean) + SERVO_YN_OFFSET, SERVO_YN);
			  DegresToCCR(90.0f + AmonDrone.Roll + (AmonDrone.RollMean) + SERVO_YP_OFFSET, SERVO_YP);


			  break;

		  /*/ Dron landed (after flying) /*/
		  case STATUS_FLY_OVER:

			  if (TVCServoEnableFlag == 1) TVCServoDisable();
			  if (EDFEnableFlag == 1) EDFDisable();

			  if (Reg200HzLoopEN == 1) // Timer 4
			  {
				  MPU6050_ReadAllDirect(&mpu6050, &hi2c3);
			  	  MPU6050_RawToDeg(&mpu6050, &AmonDrone); // calculate data to pitch and roll (and yaw)
			  	  Reg200HzLoopEN = 0;
			  }

			  if (Reg50HzLoopEN == 1)	// Timer 5
			  {
 				  uint8_t dataRdy = 0;
  				  while(dataRdy == 0)
  				  {
  					  VL53L1X_CheckForDataReady(&vl53l1Dev, &hi2c3, &dataRdy);
  				  }
  				  dataRdy = 0;
 				  VL53L1X_GetDistance(&vl53l1Dev, &hi2c3, &AmonDrone.Height);
  				  VL53L1X_ClearInterrupt(&vl53l1Dev, &hi2c3);
 				  // Read other sensors...
  				  // save to sd...
  				  Reg50HzLoopEN = 0;
			  }


			  if (Reg1HzLoopEN == 1)	// Timer 6
			  {

				  Reg1HzLoopEN = 0;
			  }


			  if (NewGPSData == 1 && USE_GPS == 1){	// new gps data available -> decode


				  NewGPSData = 0;
			  }
			  break;

		  /*/ Wrong state /*/
		  default:
			  AmonDrone.DroneStatus = STATUS_ERROR;
			  TVCServoDisable();
			  EDFDisable();
			  break;
		  }
	  }


	  /* ********* GYROSCOPE Calibration *********
	   * Before using gyroscope is necessary to set right offset values for all three accelerometers.
	   * This is set in MPU6050.h file (#define X_ACCEL_OFFSET, X_ACCEL_OFFSET, X_ACCEL_OFFSET)
	   * To get these values change CALIBRATION value to 1 and reupload code on board
	   * Disconnect everything from the board and unmount it from drone.
	   * On GPS port connect FTDI module to read UART communication. DO NOT CONNECT POWER PIN (3.3V) IF YOU ARE PLANNING TO POWER THE BOARD FROM BATTERY !!!
	   * Open serial port on PC and connect to FTDI. After initialization the dron will start to read data from gyroscope and send it over UART.
	   * Rotate board in all three directions (flat, vertical and sideways) at 90deg. In every position the one value will be around 1.
	   * The difference value and 1 is the offset value. Enter that value in firmware as offset.
	   * Change GYRO_CALIB back to 0, reupload code and mount board bact to drone
	   */

	  if (CALIBRATION == 1 )
	  {
		  StatusLED(AmonDrone.DroneStatus);

		  if (Reg200HzLoopEN == 1 && (CAL_GYRO_X == 1 || CAL_GYRO_Y == 1 || CAL_GYRO_Z == 1 || CAL_ACCEL_X == 1 || CAL_ACCEL_Y == 1 || CAL_ACCEL_Z == 1)){

			  MPU6050_ReadAllDirect(&mpu6050, &hi2c3); // Read all raw data from sensor
		  	  MPU6050_RawToDeg(&mpu6050, &AmonDrone); // calculate data to pitch and roll (and yaw)

		  	  Reg200HzLoopEN = 0;
		  }


		  if (Reg50HzLoopEN == 1 && CAL_LIDAR == 1){

			  uint8_t dataRdy = 0;
			  while(dataRdy == 0){
				  VL53L1X_CheckForDataReady(&vl53l1Dev, &hi2c3, &dataRdy);
			  }
			  dataRdy = 0;
			  VL53L1X_GetDistance(&vl53l1Dev, &hi2c3, &AmonDrone.Height);
			  VL53L1X_ClearInterrupt(&vl53l1Dev, &hi2c3);

			  Reg50HzLoopEN = 0;
		  }

		  if (GyroCalibTrig == 1){

			  // Print data
			  char message[50] = {'\0'};

			  if (CAL_ACCEL_X == 1 || CAL_ACCEL_Y == 1 || CAL_ACCEL_Z == 1) sprintf((char *)message, "Accel X = %.3f , Y = %.3f, Z = %.3f\r\n", mpu6050.ACCEL_X, mpu6050.ACCEL_Y, mpu6050.ACCEL_Z);

			  if (CAL_GYRO_X == 1 || CAL_GYRO_Y == 1 || CAL_GYRO_Z == 1) sprintf((char *)message, "Gyro X = %.3f , Y = %.3f, Z = %.3f\r\n", mpu6050.GYRO_X, mpu6050.GYRO_Y, mpu6050.GYRO_Z);

		  	  if (CAL_PITCH == 1 && CAL_ROLL == 1) sprintf((char *)message, "Pitch = %.3f, Roll = %.3f\r\n", AmonDrone.Pitch, AmonDrone.Roll);

		  	  if (CAL_ROLL == 1) sprintf((char *)message, "Roll = %.3f\r\n", AmonDrone.Roll);

		  	  if (CAL_PITCH == 1) sprintf((char *)message, "Pitch = %.3f\r\n", AmonDrone.Pitch);

		  	  if (CAL_LIDAR == 1) sprintf((char *)message, "Height = %dmm\r\n", AmonDrone.Height);

	  	  	  HAL_UART_Transmit(&huart4, (uint8_t*)message, strlen((char*)message), 100);

	  	  	  GyroCalibTrig = 0;

		  }
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 500;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 500;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2399;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 500;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 500;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2399;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 500;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 600;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 6000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Brd_Pin|CS_Flash_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_Ext_Pin|CS_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Red_Pin|LED_White_Pin|CS_RF2_Pin|EN_RF2_Pin
                          |EN_RF1_Pin|CS_RF1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Brd_Pin CS_Flash_Pin */
  GPIO_InitStruct.Pin = LED_Brd_Pin|CS_Flash_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Ext_Pin CS_SD_Pin */
  GPIO_InitStruct.Pin = CS_Ext_Pin|CS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Red_Pin LED_White_Pin CS_RF2_Pin EN_RF2_Pin
                           EN_RF1_Pin CS_RF1_Pin */
  GPIO_InitStruct.Pin = LED_Red_Pin|LED_White_Pin|CS_RF2_Pin|EN_RF2_Pin
                          |EN_RF1_Pin|CS_RF1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_IRQ2_Pin */
  GPIO_InitStruct.Pin = RF_IRQ2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_IRQ2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_IRQ1_Pin */
  GPIO_InitStruct.Pin = RF_IRQ1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_IRQ1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * UART
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_DMA(&huart4, USART4_GPSRX, 426); // enable USART Receive again
	NewGPSData = 1;		// set flag that new data has arrived
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == RF_IRQ2_Pin)
    {
    	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
    }
}

/*
 * Enabling servos for TVC stabilization
 */
uint8_t TVCServoEnable()
{
	uint8_t status = 0;
	TVCServoEnableFlag = !TVCServoEnableFlag;

	status += HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	status += HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	status += HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	status += HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	return status;
}

/*
 * Disabling servos for TVC stabilization
 */
uint8_t TVCServoDisable()
{
	uint8_t status = 0;
	EDFEnableFlag = !EDFEnableFlag;

	status += HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	status += HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	status += HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	status += HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

	return status;
}

/*
 * Enabling EDF
 */
uint8_t EDFEnable()
{
	uint8_t status = 0;

	status += HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	return status;
}

/*
 * Disabling EDF
 */
uint8_t EDFDisable()
{
	uint8_t status = 0;

	status += HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

	return status;
}


void StatusLED(uint8_t Status)
{
	// OFF: R=2400, G=0, B=0
	// ON (bright): 500
	// ON (medium): 1000

	switch(Status){
	case 0: // STARTUP (red)
		TIM1->CCR2 = 0; // LED-RGB (blue)
		TIM1->CCR3 = 0; // LED-RGB (green)
		TIM2->CCR3 = 2400; // LED-RGB (red)
		break;

	case 1: // IDLE - no RF connection (blue)
		TIM1->CCR2 = 1000; // LED-RGB (blue)
		TIM1->CCR3 = 0; // LED-RGB (green)
		TIM2->CCR3 = 0; // LED-RGB (red)
		break;

	case 2: // IDLE - RF connected (green)
		TIM1->CCR2 = 0; // LED-RGB (blue)
		TIM1->CCR3 = 500; // LED-RGB (green)
		TIM2->CCR3 = 0; // LED-RGB (red)
		break;

	case 3: // ERROR (red + brd led on)
		TIM1->CCR2 = 0; // LED-RGB (blue)
		TIM1->CCR3 = 0; // LED-RGB (green)
		TIM2->CCR3 = 1000; // LED-RGB (red)
		break;

	case 4: // ARM (all - pink)
		TIM1->CCR2 = 100; // LED-RGB (blue)
		TIM1->CCR3 = 100; // LED-RGB (green)
		TIM2->CCR3 = 100; // LED-RGB (red)
		break;

	case 5: // FLY ()
		TIM1->CCR2 = 500; // LED-RGB (blue)
		TIM1->CCR3 = 500; // LED-RGB (green)
		TIM2->CCR3 = 0; // LED-RGB (red)
		break;

	case 6: // FLY OVER (green)
		TIM1->CCR2 = 0; // LED-RGB (blue)
		TIM1->CCR3 = 500; // LED-RGB (green)
		TIM2->CCR3 = 0; // LED-RGB (red)
		break;

	case 7: // FLY OVER (green)
		TIM1->CCR2 = RGB_Blue; // LED-RGB (blue)
		TIM1->CCR3 = RGB_Green; // LED-RGB (green)
		TIM2->CCR3 = RGB_Red; // LED-RGB (red)
		break;

	default: // DEFAULT STATE
		TIM1->CCR2 = 500; // LED-RGB (blue)
		TIM1->CCR3 = 500; // LED-RGB (green)
		TIM2->CCR3 = 500; // LED-RGB (red)
		break;
	}
}


/*
 * Reading voltage of main board battery
 */
uint16_t ADC_Read_Main_Battery()
{
	uint16_t adcVal = ADC_BAT_Val[1];

	float temp = ((float)adcVal * 3.3) / 4095;

	float voltage = (((100000+10000)/10000) * temp);

	return (uint16_t)(voltage*100);
}


/*
 * Reading voltage of EDF battery
 */
uint16_t ADC_Read_EDF_Battery()
{
	uint16_t adcVal = ADC_BAT_Val[0];

	float temp = ((float)adcVal * 3.3) / 4095;

	float voltage = (((100000+10000)/10000) * temp);

	return (uint16_t)(voltage*100);
}


/*
 * DMA data from ADC ready
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	ADC_DMA_DataRdy = 1;
}


/* Regulator loop interrupt - 50Hz
 * 	- Read sensors
 * 	- Calculate TVC
 * 	- Regulate servos
 * 	- send over RF
 * 	- save to flash
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	/* TIMER 4 - 50Hz */

	if (htim->Instance == TIM4){

		Reg50HzLoopEN = 1;
		// Idle - single blink
		if (DroneStatusLocal != STATUS_STARTUP && DroneStatusLocal != STATUS_ERROR && DroneStatusLocal != STATUS_FLY && CALIBRATION == 0)
		{
			if (LED_blink_cnt_ON < 50) // LED OFF
			{
				LED_blink_cnt_ON++;
			}
			else
			{
				if (LED_blink_cnt_OFF < 10) // LED ON for short time
				{
					if (LED_blink_cnt_OFF == 0)
					{
						HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_SET);
					}
					LED_blink_cnt_OFF++;
				}
				else
				{
					HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_RESET);
					LED_blink_cnt_OFF = 0;
					LED_blink_cnt_ON = 0;
				}
			}
		}


		// Fly - dual blink
		if (DroneStatusLocal != STATUS_STARTUP && DroneStatusLocal != STATUS_ERROR && DroneStatusLocal == STATUS_FLY && CALIBRATION == 0)
		{
			if (LED_blink_cnt_ON < 50) // LED OFF
			{
				LED_blink_cnt_ON++;
			}
			else
			{
				if (LED_blink_cnt_OFF < 10) // LED ON for short time
				{
					if (LED_blink_cnt_OFF <3)
					{
						HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_SET);
					}

					if (LED_blink_cnt_OFF >=3 && LED_blink_cnt_OFF <7)
					{
						HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_RESET);
					}

					if (LED_blink_cnt_OFF >=7)
					{
						HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_SET);
					}

					LED_blink_cnt_OFF++;
				}
				else
				{
					HAL_GPIO_WritePin(LED_White_GPIO_Port, LED_White_Pin, GPIO_PIN_RESET);
					LED_blink_cnt_OFF = 0;
					LED_blink_cnt_ON = 0;
				}
			}
		}


		/* GYRO Calibration */

		if (GyroCalibTrig == 0 && CALIBRATION == 1)
		{
			if (LED_blink_cnt_ON >= 5)
			{
				LED_blink_cnt_ON = 0;
				GyroCalibTrig = 1;
			}

			LED_blink_cnt_ON++;
		}
	}


	/* TIMER 5 - 200Hz */

	if (htim->Instance == TIM5)
	{
		Reg200HzLoopEN = 1;

		/* RGB Cycling */
		// Red
		if (RGB_RedMax == 0)
		{
			RGB_Red += 1;
			if (RGB_Red == 1000) RGB_RedMax = 1;
		}

		if (RGB_RedMax == 1)
		{
			RGB_Red -= 1;
			if (RGB_Red == 0) RGB_RedMax = 0;
		}

		// Green
		if (RGB_GreenMax == 0 || (RGB_Green == 300 && RGB_GreenMax == 0 ))
		{
			RGB_Green += 1;
			if (RGB_Green == 900) RGB_GreenMax = 1;
		}

		if (RGB_GreenMax == 1)
		{
			RGB_Green -= 1;
			if (RGB_Green == 0) RGB_GreenMax = 0;
		}

		// Blue
		if (RGB_BlueMax == 0 || (RGB_Blue == 600 && RGB_BlueMax == 0 ))
		{
			RGB_Blue += 1;
			if (RGB_Blue == 900) RGB_BlueMax = 1;
		}

		if (RGB_BlueMax == 1)
		{
			RGB_Blue -= 1;
			if (RGB_Blue == 0) RGB_BlueMax = 0;
		}

	}

	/* TIMER 6 - 1Hz */
	if (htim->Instance == TIM6){
		Reg1HzLoopEN = 1;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
