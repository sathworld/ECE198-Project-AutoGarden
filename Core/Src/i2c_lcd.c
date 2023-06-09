
/** Put this in the src folder **/

#include "i2c_lcd.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd (LCD_CLEARDISPLAY);
	HAL_Delay(2);
}

void lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_home(){
	lcd_send_cmd (LCD_RETURNHOME);
	//HAL_Delay(2);
}
void lcd_no_display(){
	_displaycontrol &= ~LCD_DISPLAYON;
	lcd_send_cmd (LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_display(){
	_displaycontrol |= LCD_DISPLAYON;
	lcd_send_cmd (LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_no_blink(){
    _displaycontrol &= ~LCD_BLINKON;
    lcd_send_cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_blink(){
    _displaycontrol |= LCD_BLINKON;
    lcd_send_cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_no_cursor(){
    _displaycontrol &= ~LCD_CURSORON;
    lcd_send_cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_cursor(){
    _displaycontrol |= LCD_CURSORON;
    lcd_send_cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_scroll_display_left(){
	lcd_send_cmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void lcd_scroll_display_right(){
	lcd_send_cmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void lcd_left_to_right(){
    _displaymode |= LCD_ENTRYLEFT;
    lcd_send_cmd(LCD_ENTRYMODESET | _displaymode);
}
void lcd_right_to_left(){
    _displaymode &= ~LCD_ENTRYLEFT;
    lcd_send_cmd(LCD_ENTRYMODESET | _displaymode);
}

void lcd_autoscroll(){
    _displaymode |= LCD_ENTRYSHIFTINCREMENT;
    lcd_send_cmd(LCD_ENTRYMODESET | _displaymode);
}
void lcd_no_autoscroll(){
    _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
    lcd_send_cmd(LCD_ENTRYMODESET | _displaymode);
}
void lcd_create_char(uint8_t location, uint8_t charmap[]){
    location &= 0x7; // we only have 8 locations 0-7
    lcd_send_cmd(LCD_SETCGRAMADDR | (location << 3));
    for (uint8_t i = 0; i != 8; i++)
    	lcd_send_data(charmap[i]);
}
void lcd_set_cursor(uint8_t col, uint8_t row){
    int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if (row > 4)
        row = 4 - 1; // we count rows starting w/0
    lcd_send_cmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}
