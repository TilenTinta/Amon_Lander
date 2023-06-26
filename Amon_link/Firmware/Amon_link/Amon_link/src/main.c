/*
 * main.c
 *
 * Created: 2/21/2023 9:42:00 PM
 *  Author: Tinta T.
 */ 


#include <xc.h>
#include "../inc/main.h"

/** Timer_1 IRQ **/
ISR(TIMER1_COMPA_vect)
{
	// Startup init
	if (InitOK == 0){
		if (InitTime >= 30){
			InitOK = 1;
			InitTime = 0;
			GPIO_PIN_Write(GPIO_PORT_B,LED_Red,GPIO_HIGH);
		}
		else
		{
			InitTime++;
			GPIO_PIN_Toggle(GPIO_PORT_B,LED_Red); // Blink red led
		}
	}
	
	// Pair sequence (trigered after startup with button)
	if(PairTrig == 1){
		if (PairTime >= 30){
			PairTrig = 0;
			PairTime = 0;
		}else{
			PairTime++;
			GPIO_PIN_Toggle(GPIO_PORT_B,LED_Red); // Blink red led
		}
		
	}
	
}


///****** MAIN BEGIN ******///
int main(void)
{	
	_delay_ms(100); // Init delay for all devices power on
	
	InitError = DeviceInit();	// Initialize all devices and I/O
	
	uint8_t DataToSend[34] = {"********* Amon Link *********\r\nff"};
	USART_DATA_Transmit(DataToSend);
		
	while(1)
	{
		// No error in init and after init time
		if (InitError == 0 && InitOK == 1 && Pair_Status == 1){					// OK
			
			GPIO_PIN_Write(GPIO_PORT_B,LED_Green,GPIO_LOW); // Green led on
			

		}else if(InitError == 0 && InitOK == 1 && Pair_Status == 0){			// OK, NOPAIR
			
			GPIO_PIN_Write(GPIO_PORT_B,LED_Blue,GPIO_LOW); // Blue led on
			
			_delay_ms(1000);
			SPI_test(RF_CS1);

						
			if(((PINC & (1 << 2)) >> 2) == 1){	// Repete pairing
				//PairTrig = 1;
			}
			
		}else if(InitError == 1 && InitOK == 1){			// ERROR
			
			

		}
	}
}
///****** END MAIN ******///



/* Initialization function for device */
uint8_t DeviceInit()
{
	uint8_t error = 0;
	C
	
	// GPIO
	DDRB |= (1 << RF_SCK) |(1 << RF_MOSI) | (1 << LED_Green) | (1 << LED_Blue) | (1 << LED_Red) ; // set port B 
	DDRC = 0b00000000; // set port C I/O
	DDRD |= (1 << RF_CS2) | (1 << RF_EN2) | (1 << RF_EN1) | (1 << RF_CS1) ; // set port D I/O
	GPIO_PIN_Write(GPIO_PORT_B,LED_Blue,GPIO_HIGH);
	GPIO_PIN_Write(GPIO_PORT_B,LED_Green,GPIO_HIGH);
	
	
	// IRQ
	IRQ_Timer1_Init();
	USART_Init(USART_PRESC);
	
	// SPI - RF
	SPI_Init();
	
	GPIO_PIN_Write(GPIO_PORT_D,RF_EN1,GPIO_HIGH); // set radio1 as transmitter
	GPIO_PIN_Write(GPIO_PORT_D,RF_EN2,GPIO_LOW);  // set radio2 as receiver
	
	GPIO_PIN_Write(GPIO_PORT_D,RF_CS1,GPIO_HIGH); // CS high - off
	GPIO_PIN_Write(GPIO_PORT_D,RF_CS2,GPIO_HIGH); // CS high - off
	//INT0_interrupt_Init();
	//NRF24L01_Inti1();
	//NRF24L01_Inti2();

	
	
	// PAIR
	Pair_Status = 0; // brisi
	
	sei();	// Enable all interrupts
	
	// Error return ??????????
	if(error == 0){
		return 0;
	}else{
		return 1;
	}
}

/* Init function for timer */
void IRQ_Timer1_Init()
{
	TCCR1B |= (1 << WGM12 );   // Configure timer 1 for CTC mode
	OCR1A = 25000;             // 10Hz (100mS) at 16MHz, prescaler 64
	TIMSK1 |= (1 << OCIE1A );  // Enable interrupt
	TCCR1B |= ((1 << CS10 ) | (1 << CS11 )); // Start Timer F_CPU/64
}

void INT0_interrupt_Init()
{
	
	// Set detect on falling edge
	EICRA |= (1 << ISC01);
	EICRA &= ~(1 << ISC00);
	
	// enable int
	EIMSK |= (1 << INT0); 
		
}

// Interrupt for RF RX data
ISR(INT0_vect)
{
	cli();
	
	
	
	sei();
}

