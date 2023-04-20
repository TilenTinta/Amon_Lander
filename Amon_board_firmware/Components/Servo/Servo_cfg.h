/*
 * Servo_cfg.h
 *
 *  Created on: Feb 23, 2023
 *      Author: TintaT
 */

#ifndef SERVO_SERVO_CFG_H_
#define SERVO_SERVO_CFG_H_

#include "SERVO.h"

const SERVO_CfgType SERVO_CfgParam[SERVO_NUM] =
{
    // Servo Motor 1 Configurations
    {
    GPIOA,
    GPIO_PIN_0,
    TIM2,
    &TIM2->CCR1,
    TIM_CHANNEL_1,
    72000000,
    0.65,
    2.3
    },
    // Servo Motor 2 Configurations
    {
    GPIOA,
    GPIO_PIN_3,
    TIM2,
    &TIM2->CCR4,
    TIM_CHANNEL_4,
    72000000,
    0.75,
    2.5
    },
    // Servo Motor 3 Configurations
    {
    GPIOB,
    GPIO_PIN_0,
    TIM3,
    &TIM3->CCR3,
    TIM_CHANNEL_3,
    72000000,
    0.62,
    2.4
    },
    // Servo Motor 4 Configurations
    {
        GPIOB,
    GPIO_PIN_9,
    TIM4,
    &TIM4->CCR4,
    TIM_CHANNEL_4,
    72000000,
    0.55,
    2.35
    }
};

#endif /* SERVO_SERVO_CFG_H_ */
