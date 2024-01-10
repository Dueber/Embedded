/*
 * FirstProject.c
 *
 * Created: 1/8/2024 10:42:19 AM
 * Author : Christopher Dueber
 */ 

#define F_CPU 16000000

#include <avr/io.h>

#include <util/delay.h>

#include <stdio.h>

#include "LCD_Driver.h"


//********************************************//
// On-board LEDS: PORTB
// Pull-up resistor active: 1=output, 0=input
// DDRB: Input/Output
// PINB: 
// PORTB:
//********************************************//



// PORTD Push Buttons (Pull-up active)



int main(void)
{
    DDRB = 0b11111111;

    PORTB = 0b00000000;
	
	DDRD = 0b00000000;
	
	PORTD = 0b11110000;

	LCDInit();
	LCDBacklightOn();
	
    //char data1[] = "Hello, World!";
    //char data2[] = "AVR Programming";	
	
    while (1) 
    {
		
		uint8_t LEDS = 0b10000000;
		
		for (int i = 0; i < 7; i++) {
			PORTB = LEDS;
			//LCDWrLn1(data1);
			_delay_ms(50);
			LEDS >>= 1;
		}
		
		for (int i = 0; i < 7; i++) {
			PORTB = LEDS;
			//LCDWrLn2(data2);
			_delay_ms(50);
			LEDS <<= 1;
		}
		
    }
}

