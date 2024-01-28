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


void LCD_Internal_WriteCMD(uint8_t mpr);
void LCDDelay();

void LCDInit() {
	uint8_t mpr;

	
	DDRB |= (1 << DDB2) | (1 << DDB1) | (1 << DDB0);		// set MOSI, SCL, and SS as outputs		

	
	DDRF |= (1 << DDF1);									// set lcd_A0 as output
	
	
	DDRC |= (1 << DDC7);									// enable LCD backlight control
	
	
	DDRF |= (1 << DDF0);									// set lcd_RST_N as output
	
	
	PORTF &= ~(1 << PF0);									// pull lcd_RST_N low for 1 millisecond
	LCDDelay();
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

	
	PORTB &= ~(1 << PB0);									// activate slave select

	mpr = ST7565R_SET_V_BIAS;								// Initialization commands
	LCD_Internal_WriteCMD(mpr);
	mpr = ST7565R_SEG_SCAN_DIR;
	LCD_Internal_WriteCMD(mpr);
	mpr = ST7565R_EV_MODE_SET;
	LCD_Internal_WriteCMD(mpr);
	mpr = ST7565R_EV_REG_SET;
	LCD_Internal_WriteCMD(mpr);
	mpr = ST7565R_V_REG_RATIO;
	LCD_Internal_WriteCMD(mpr);
	mpr = ST7565R_V_BOOST_RATIO;
	LCD_Internal_WriteCMD(mpr);
	mpr = (ST7565R_DISPLAY_ENABLE | (1 << 0));
	LCD_Internal_WriteCMD(mpr);

	
	PORTB |= (1 << PB0);									// deactivate slave select
	
}

void LCDBacklightOn() {
	
	PORTC |= (1 << PC7);									// Enable backlight
	
}

void LCDBacklightOff() {
	
	PORTC |= (1 >> PC7);									// Disable backlight
	
}

// ******************************************************
// This functions pull pin A0 low before writing to 
// the controller. Used for initializing LCD screen by
// sending the appropriate commands
//
// ******************************************************

void LCD_Internal_WriteCMD(uint8_t mpr) {

	SPDR_REG = mpr;
	
	PORTF &= ~(1 << PF1);									// Ensure A0 is driven low
	while (!(SPSR_REG & (1 << SPIF)));
}

void SPI_MasterTransmit(char cData) {
	
	SPDR = cData;
	
	PORTF &= ~(1 << PF1);									// Ensure A0 is driven low
	while(!(SPSR & (1 << SPIF)));
}

void LCDDelay() {
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

void LCD_WriteLn(uint16_t* FontBitmap, uint8_t r17) {
	// Set page address
	LCD_Internal_WriteCMD(ST7565R_SET_PAGE_ADDR | (r17 & 0x0F)); // adjust r17 if needed

	// Set column address (high nibble)
	LCD_Internal_WriteCMD(ST7565R_COLUMN_ADDR_H);

	// Set column address (low nibble)
	LCD_Internal_WriteCMD(ST7565R_COLUMN_ADDR_L);

	// Write data to display RAM
	for (int i = 0; i < 32; i++) {
		LCD_Internal_WriteCMD(FontBitmap[i] >> 8); // high byte
		LCD_Internal_WriteCMD(FontBitmap[i] & 0xFF); // low byte
	}
}

void LCDClr() {
	
	// Clear entire LCD screen code
	
}


#endif /* LCD_DRIVER_H_ */
