/*
 * Servo.h
 *
 *  Created on: Feb 23, 2023
 *      Author: TintaT
 */

#ifndef SERVO_SERVO_H_
#define SERVO_SERVO_H_

#include "stm32f4xx_hal.h"					/* For PWM */

/*
 * PWM: 50Hz (20ms)
 *  - DutyCycle: 0deg - 0.5ms		(1518)
 *  - DutyCycle: 90deg - 1.5ms		(4555)
 *  - DutyCycle: 180deg - 2.5ms		(7592)
 *
 * 	1deg = 0.011ms
 * 	1deg = 33.4 value
 * 	0.1deg = 3.374 value
 *
 */

#define SERVO_XP 	1
#define SERVO_XN 	2
#define SERVO_YP 	3
#define SERVO_YN 	4
#define PWM_EDF 	5

#define SERVO_XP_OFFSET 	-3
#define SERVO_XN_OFFSET 	1
#define SERVO_YP_OFFSET 	-3
#define SERVO_YN_OFFSET 	0

void DegresToCCR(float Degress, uint8_t Servo);


#endif /* SERVO_SERVO_H_ */
