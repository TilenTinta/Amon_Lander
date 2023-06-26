/*
 * main.h
 *
 * Created: 26/02/2023 22:25:05
 *  Author: Tinta T.
 */ 

#ifndef MAIN_H_
#define MAIN_H_

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz clock speed
#endif

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "../inc/LL_Fcn.h"
#include "../inc/nRF24L01.h"
#include "../inc/USART.h"

// Values in () are connections on Arduino Nano
#define LED_Red		PINB0			// Led (D8)
#define LED_Blue	PINB1			// Led (D9)
#define LED_Green	PINB2			// Led (D10)

#define SW_RF_NUM_1	PINC0			// Switch for number of radios (A0)
#define SW_RF_NUM_2	PINC1			// Switch for number of radios (A1)
#define BTN_Pair	PINC2			// Button for pairing (A2)

#define RF_MOSI		PINB3			// SPI - MOSI (D11)
#define RF_MISO		PINB4			// SPI - MISO (D12)
#define RF_SCK		PINB5			// SPI - SCK (D13)
#define RF_CS1		PIND3			// SPI - CS radio 1 (D3)
#define RF_CS2		PIND7			// SPI - CS radio 2 (D7)
#define RF_IRQ1		PIND2			// SPI - interrupt radio 1 (D2)
#define RF_IRQ2		PIND5			// SPI - interrupt radio 2 (D5)
#define RF_EN1		PIND4			// SPI - enable 1 (D4)
#define RF_EN2		PIND6			// SPI - enable 2 (D6)

// Variables
uint8_t InitError = 1;
uint8_t InitTime = 0;
uint8_t InitOK = 0;
uint8_t PairTime = 0;
uint8_t PairTrig = 0;
char USART_DATA_Buffer_Transmit[100] = {};
char USART_DATA_Buffer_Recieve[100] = {};
	
uint8_t Pair_Status = 0;

/* Main functions */
uint8_t DeviceInit();
void IRQ_Timer1_Init();
void INT0_interrupt_Init();

#endif /* MAIN_H_ */