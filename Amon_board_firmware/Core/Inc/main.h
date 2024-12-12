/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "BME280.h"
#include "PWM.h"
#include "GPS.h"
#include "DroneData.h"

#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EDF_BAT_Pin GPIO_PIN_0
#define EDF_BAT_GPIO_Port GPIOA
#define BRD_BAT_Pin GPIO_PIN_1
#define BRD_BAT_GPIO_Port GPIOA
#define TIM2_CH3_RGB_B_Pin GPIO_PIN_2
#define TIM2_CH3_RGB_B_GPIO_Port GPIOA
#define LED_Brd_Pin GPIO_PIN_3
#define LED_Brd_GPIO_Port GPIOA
#define CS_Flash_Pin GPIO_PIN_4
#define CS_Flash_GPIO_Port GPIOA
#define CS_Ext_Pin GPIO_PIN_4
#define CS_Ext_GPIO_Port GPIOC
#define CS_SD_Pin GPIO_PIN_5
#define CS_SD_GPIO_Port GPIOC
#define TIM3_CH3_Y__Pin GPIO_PIN_0
#define TIM3_CH3_Y__GPIO_Port GPIOB
#define TIM3_CH4_Y__Pin GPIO_PIN_1
#define TIM3_CH4_Y__GPIO_Port GPIOB
#define LED_Red_Pin GPIO_PIN_2
#define LED_Red_GPIO_Port GPIOB
#define LED_White_Pin GPIO_PIN_10
#define LED_White_GPIO_Port GPIOB
#define TIM2_CH4_EDF_Pin GPIO_PIN_11
#define TIM2_CH4_EDF_GPIO_Port GPIOB
#define TIM3_CH1_X__Pin GPIO_PIN_6
#define TIM3_CH1_X__GPIO_Port GPIOC
#define TIM3_CH2_X__Pin GPIO_PIN_7
#define TIM3_CH2_X__GPIO_Port GPIOC
#define TIM1_CH2_RGB_G_Pin GPIO_PIN_9
#define TIM1_CH2_RGB_G_GPIO_Port GPIOA
#define TIM1_CH3_RGB_R_Pin GPIO_PIN_10
#define TIM1_CH3_RGB_R_GPIO_Port GPIOA
#define UART4_TX_GPS_Pin GPIO_PIN_10
#define UART4_TX_GPS_GPIO_Port GPIOC
#define UART4_RX_GPS_Pin GPIO_PIN_11
#define UART4_RX_GPS_GPIO_Port GPIOC
#define RF_IRQ2_Pin GPIO_PIN_2
#define RF_IRQ2_GPIO_Port GPIOD
#define CS_RF2_Pin GPIO_PIN_4
#define CS_RF2_GPIO_Port GPIOB
#define EN_RF2_Pin GPIO_PIN_5
#define EN_RF2_GPIO_Port GPIOB
#define RF_IRQ1_Pin GPIO_PIN_7
#define RF_IRQ1_GPIO_Port GPIOB
#define RF_IRQ1_EXTI_IRQn EXTI9_5_IRQn
#define EN_RF1_Pin GPIO_PIN_8
#define EN_RF1_GPIO_Port GPIOB
#define CS_RF1_Pin GPIO_PIN_9
#define CS_RF1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
