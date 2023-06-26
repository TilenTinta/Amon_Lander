/*
 * LL_Fcn.h
 *
 * Created: 27/02/2023 23:07:04
 *  Author: Tinta T.
 */ 

#ifndef LL_FCN_H_
#define LL_FCN_H_

#include <avr/io.h>
#include <xc.h>

/* DDIR setup */
#define GPIO_PORT_B 1
#define GPIO_PORT_C 2
#define GPIO_PORT_D 3

#define GPIO_IN 0
#define GPIO_OUT 1

/* PIN state setup */
#define GPIO_LOW 0
#define GPIO_HIGH 1

void GPIO_PIN_SETUP(char port, uint8_t pinNum, char fcn);
void GPIO_PIN_Write(char port, uint8_t pinNum, char state);
void GPIO_PIN_Toggle(char port, uint8_t pinNum);
uint8_t GPIO_PIN_Read(char port, uint8_t pinNum);

#endif /* LL_FCN_H_ */