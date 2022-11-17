#ifndef I2C_LCD
#define I2C_LCD

#include "stm32f4xx_hal.h"

#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0B00000100 // Enable bit
#define Rw 0B00000010 // Read/Write bit
#define Rs 0B00000001 // Register select bit

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_clear (void);

void lcd_home();
void lcd_no_display();
void lcd_display();
void lcd_no_blink();
void lcd_blink();
void lcd_no_cursor();
void lcd_cursor();
void lcd_scroll_display_left();
void lcd_scroll_display_right();
void lcd_print_left();
void lcd_print_right();
void lcd_left_to_right();
void lcd_right_to_left();
void lcd_shift_increment();
void lcd_shift_decrement();
void lcd_no_backlight();
void lcd_backlight();
void lcd_autoscroll();
void lcd_no_autoscroll();
void lcd_create_char(uint8_t, uint8_t[]);
void lcd_set_cursor(uint8_t, uint8_t);

extern uint8_t _Addr;
extern uint8_t _displayfunction;
extern uint8_t _displaycontrol;
extern uint8_t _displaymode;
extern uint8_t _numlines;
extern uint8_t _cols;
extern uint8_t _rows;
extern uint8_t _backlightval;
#endif
