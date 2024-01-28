#ifndef LCD_CONFIG_H_
#define LCD_CONFIG_H_

#define ST7565R_USART_SPI_INTERFACE
#define ST7565R_USART_SPI				&USARTD0

#define ST7565R_CLOCK_SPEED				16000000						// CPU clock speed 16 MHz

#define ST7565R_DISPLAY_CONTRAST_MAX	40
#define ST7565R_DISPLAY_CONTRAST_MIN	30

#define ST7565R_A0_PIN					NHD_C12832A1Z_REGISTER_SELECT	// Select command [0] or data mode [1]
#define ST7565R_CS_PIN					NHD_C12832A1Z_CSN				 
#define ST7565R_RESET_PIN				NHD_C12832A1Z_RESETN

#define ST7565R_DISPLAY_ENABLE			0b10101110						// Display ON/OFF	
#define ST7565R_DISPLAY_START_LN		0b01000000						// Display Start Line Set
#define ST7565R_SET_PAGE_ADDR			0b10110000						// Page Address Set
#define ST7565R_COLUMN_ADDR_H			0b00010000						// MSB 4-bit display
#define ST7565R_COLUMN_ADDR_L			0b00000000						// LSB 4-bit display
#define ST7565R_STATUS_READ				0b00000000						// Reads the status data [bits 5:8]
#define ST7565R_ADC_SELECT				0b10100000						// Normal SEG output from RAM address 
#define ST7565R_DISPLAY_SELECT			0b10100110						// LCD display normal
#define ST7565R_DISPLAY_ALL				0b10100100						// Display all points, normal
#define ST7565R_SET_V_BIAS				0b10100010						// LCD Bias Set, voltage bias ratio (1/6 bias, 1/33 duty)
#define ST7565R_SEG_SCAN_DIR			0b10100000						// Segment Driver direction
#define ST7565R_V_REG_RATIO				0b00100010						// Resistor ratio for voltage regulator 
#define ST7565R_V_BOOST_RATIO			0b00000111						// LOOK FURTHER
#define ST7565R_EV_MODE_SET				0b10000001						// Electronic Volume Mode Set
#define ST7565R_EV_REG_SET				0b00001111						// Electronic Volume Register (table 10)


#endif
