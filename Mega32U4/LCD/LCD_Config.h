#ifndef LCD_CONFIG_H_
#define LCD_CONFIG_H_

#define LCD_DISPLAY_ON					0b10101111
#define LCD_DISPLAY_OFF					0b10101110
#define LCD_DISPLAY_NORMAL				0b10100100	
#define LCD_DISPLAY_ALL					0b10100101

#define LCD_PAGE_ADDR_SET				0b10110000			
#define LCD_CLMN_ADDR_SET_UPPER			0b00010000
#define LCD_CLMN_ADDR_SET_LOWER			0b00000000

#define LCD_MODE_NORMAL					0b10100000
#define LCD_MODE_REVERSE				0b10100001
#define LCD_MODE_RESET					0b11100010

#define LCD_SET_START_LINE				(1<<6)

void LCD_INIT (void);

void LCD_BACKLIGHT_ON (void);

void LCD_BACKLIGHT_OFF (void);

void LCD_DELAY (void);

void LCD_SEND_COMMAND (uint8_t data);

void LCD_ALL_ON (void);

void LCD_CLEAR_SCREEN (void);

void LCD_NORMAL (void);

void LCD_SET_COLUMN_UPPER (uint8_t addr);

void LCD_SET_COLUMN_LOWER (uint8_t addr);

void LCD_SET_START_WRITE (uint8_t addr);

#endif
