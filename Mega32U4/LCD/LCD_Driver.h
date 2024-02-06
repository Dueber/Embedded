/*
 * LCD_Driver.h
 *
 * Created: 1/8/2024 12:36:51 PM
 *  Author: Christopher Dueber
 */ 


#ifndef LCD_DRIVER_H_
#define LCD_DRIVER_H_

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "LCD_Config.h"


#define SPCR_REG (*((volatile unsigned char *)(0x2D)))		// Register assignments
#define SPSR_REG (*((volatile unsigned char *)(0x2E)))
#define SPDR_REG (*((volatile unsigned char *)(0x2F)))



void LCD_INIT() {

	DDRB |= (1 << DDB2) | (1 << DDB1) | (1 << DDB0);		// set MOSI, SCL, and SS as outputs		

	DDRF |= (1 << DDF1);									// set lcd_A0 as output
	
	DDRC |= (1 << DDC7);									// enable LCD_BACKLIGHT control
	
	DDRF |= (1 << DDF0);									// set lcd_RST_N as output
	
	PORTF &= ~(1 << PF0);									// pull lcd_RST_N low for 1 millisecond
	LCD_DELAY ();
	PORTF |= (1 << PF0);

	// SPCR Setup: SPI Control Register
	// SPIE: Enable SPI interrupt vector
	// SPE: SPI Enable
	// MSTR: Master SPI mode
	// CPOL: Idle CLK = 1, Falling -> Rising
	// CPHA: Setup -> Sample data
	SPCR_REG = (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA);
	
	// SPSR Setup: SPI Status Register
	// SPI2X: CLK freq. is doubled, see clock speed reference
	// SPIF: SPI Interrupt Vector flag
	SPSR_REG = (1 << SPIF) | (1 << SPI2X);
	
	LCD_BACKLIGHT_ON();

}

// ********************************************************
// This function is used on LCD initialization to turn on
// the built-in backlight for the LCD screen.
// ********************************************************

void LCD_BACKLIGHT_ON (void) {
	
	PORTC |= (1 << PC7);				// Enable backlight
	
}


// ******************************************************
// This function is used on LCD initialization to turn 
// off the built-in backlight for the LCD screen.
// ******************************************************

void LCD_BACKLIGHT_OFF (void) {
	
	PORTC |= (1 >> PC7);				// Disable backlight
	
}


// ******************************************************
// Page address of the display data RAM is specified
// through the Page Address Set Command.  This command 
// specifies the page address corresponding to the low 
// address when the MPU accesses the display data RAM
// ******************************************************

void LCD_SET_PAGE_ADDR (void) {
	
	LCD_SEND_COMMAND(LCD_PAGE_ADDR_SET);
	
}

// ******************************************************
// The display data RAM column address is specified by 
// the Column Address Set command. The specified column
// address is incremented (+1) with each display data
// read/write command.
// ******************************************************

void LCD_SET_COLUMN_UPPER (void) {
	
	LCD_SEND_COMMAND(LCD_CLMN_ADDR_SET_UPPER);
	
}

void LCD_SET_COLUMN_LOWER (void) {
	
	LCD_SEND_COMMAND(LCD_CLMN_ADDR_SET_LOWER);
	
}


// ******************************************************
// The line address circuit specifies the line address
// relating to the COM output when the contents of
// the display data RAM are displayed.
// ******************************************************

void LCD_SET_LINE_ADDR (void) {
	
	LCD_SEND_COMMAND(LCD_LINE_ADDR_SET);
	
}

// ******************************************************
// This functions pull pin A0 low before writing to
// the controller. Used for initializing LCD screen by
// sending the appropriate commands.
// ******************************************************

void LCD_BEGIN_CMD_SIGNAL(void) {
	
	PORTF &= ~(1 << PF1);									// A0 is driven low

}


// ******************************************************
// This functions pull pin A0 high before sending data to
// the screen.
// ******************************************************

void LCD_BEGIN_TX_SIGNAL(void) {
	
	PORTF &= (1 << PF1);									// A0 is set
	
}



void LCD_SEND_COMMAND (uint8_t data) {
	
	SPDR_REG = data;										// initiate transmission
	
	LCD_BEGIN_CMD_SIGNAL();									// Put the LCD into command mode

	while (!(SPSR_REG & (1 << SPIF))) {}					// Wait for SPI to finish
		
}	
	







void LCD_DELAY (void) {
	
	_delay_ms(1);											// Wait for 1 ms
	
	}



// *************************************************** //
// When the MPU writes data to the display data RAM, 
// once the data is stored in the bus holder, then it is
// written to the display data RAM before the next data write
// cycle. Moreover, when the MPU reads the display data RAM,
// the first data read cycle (dummy) stores the read data in the
// bus holder, and then the data is read from the bus holder to
// the system bus at the next data read cycle.
//
// TODO:
//  - Write data to bus holder
//  - Write data to display RAM
//  - Read dummy read cycle from data RAM
//  - Read data from bus holder to system bus ?
//  
// NOTES:
//  - r17 destination bank in LCD RAM
//  - Bitmap + destination -> send data to LCD
// *************************************************** //




#endif /* LCD_DRIVER_H_ */
