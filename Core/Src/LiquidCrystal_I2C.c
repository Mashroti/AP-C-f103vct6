// Based on the work by DFRobot

#include "LiquidCrystal_I2C.h"
#include <inttypes.h>
#include "main.h"

#define delay(Delay)				HAL_Delay(Delay)
#define	delayMicroseconds(Delay)	delay_us(Delay)

extern I2C_HandleTypeDef hi2c;


void delay_us(unsigned int Delay)
{
	long long i,k;
	k = (FCLK/21000000)*Delay;
	for(i=0 ; i<k ; i++);
}

void lcd_init(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows){
	_Addr = lcd_Addr;
	_cols = lcd_cols;
	_rows = lcd_rows;
	_backlightval = LCD_NOBACKLIGHT;
	lcd_init_priv();
}

void lcd_init_priv()
{
	_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	lcd_begin(_cols, _rows);
}

void lcd_begin(uint8_t cols, uint8_t lines) {
	if (lines > 1) {
		_displayfunction |= LCD_2LINE;
	}
	_numlines = lines;

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending lcd_commands. Arduino can turn on way befer 4.5V so we'll wait 50
	delay(50); 
  
	// Now we pull both RS and R/W low to begin lcd_commands
	lcd_expanderWrite(_backlightval);	// reset expanderand turn backlight off (Bit 8 =1)
	delay(50);

  	//put the LCD into 4 bit mode
	// this is according to the hitachi HD44780 datasheet
	// figure 24, pg 46
	
	  // we start in 8bit mode, try to set 4 bit mode
   lcd_write4bits(0x03 << 4);
   delayMicroseconds(4500); // wait min 4.1ms
   
   // second try
   lcd_write4bits(0x03 << 4);
   delayMicroseconds(4500); // wait min 4.1ms
   
   // third go!
   lcd_write4bits(0x03 << 4);
   delayMicroseconds(150);
   
   // finally, set to 4-bit interface
   lcd_write4bits(0x02 << 4);


	// set # lines, font size, etc.
	lcd_command(LCD_FUNCTIONSET | _displayfunction);
	
	// turn the display on with no cursor or blinking default
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	lcd_display();
	
	// clear it off
	lcd_clear();
	
	// Initialize to default text direction (for roman languages)
	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	
	// set the entry mode
	lcd_command(LCD_ENTRYMODESET | _displaymode);
	
	lcd_home();
  
}

/********** high level lcd_commands, for the user! */
void lcd_clear(){
	lcd_command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	delayMicroseconds(2000);  // this lcd_command takes a long time!
}

void lcd_home(){
	lcd_command(LCD_RETURNHOME);  // set cursor position to zero
	delayMicroseconds(2000);  // this lcd_command takes a long time!
}

void lcd_setCursor(uint8_t col, uint8_t row){
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if ( row > _numlines ) {
		row = _numlines-1;    // we count rows starting w/0
	}
	lcd_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void lcd_noDisplay() {
	_displaycontrol &= ~LCD_DISPLAYON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_display() {
	_displaycontrol |= LCD_DISPLAYON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor() {
	_displaycontrol &= ~LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_cursor() {
	_displaycontrol |= LCD_CURSORON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_noBlink() {
	_displaycontrol &= ~LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_blink() {
	_displaycontrol |= LCD_BLINKON;
	lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These lcd_commands scroll the display without changing the RAM
void lcd_scrollDisplayLeft(void) {
	lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void lcd_scrollDisplayRight(void) {
	lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcd_leftToRight(void) {
	_displaymode |= LCD_ENTRYLEFT;
	lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void lcd_rightToLeft(void) {
	_displaymode &= ~LCD_ENTRYLEFT;
	lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void lcd_autoscroll(void) {
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void lcd_noAutoscroll(void) {
	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcd_createChar(uint8_t location, uint8_t charmap[]) {
	location &= 0x7; // we only have 8 locations 0-7
	lcd_command(LCD_SETCGRAMADDR | (location << 3));
	for (int i=0; i<8; i++) {
		lcd_write(charmap[i]);
	}
}

//createChar with PROGMEM input

void lcd_puts(const char* s)
{
	while(*s)
	{
		lcd_write(*s++);
	}
}

void lcd_putch(char character)
{
	lcd_write(character);
}

void lcd_puts_XY(uint8_t x,uint8_t y, char* s)
{
	lcd_setCursor(x, y);
	lcd_puts(s);
}
// Turn the (optional) backlight off/on
void lcd_noBacklight(void) {
	_backlightval=LCD_NOBACKLIGHT;
	lcd_expanderWrite(0);
}

void lcd_backlight(void) {
	_backlightval=LCD_BACKLIGHT;
	lcd_expanderWrite(0);
}


uint8_t lcd_write(uint8_t value) {
	lcd_send(value, Rs);
	return 1;
}
/*********** mid level lcd_commands, for sending data/cmds */

inline void lcd_command(uint8_t value) {
	lcd_send(value, 0);
}


/************ low level data pushing lcd_commands **********/

// write either lcd_command or data
void lcd_send(uint8_t value, uint8_t mode) {
	uint8_t highnib=value&0xf0;
	uint8_t lownib=(value<<4)&0xf0;
       lcd_write4bits((highnib)|mode);
	lcd_write4bits((lownib)|mode);
}

void lcd_write4bits(uint8_t value) {
	lcd_expanderWrite(value);
	lcd_pulseEnable(value);
}

void lcd_expanderWrite(uint8_t _data){
	//printIIC((int)(_data) | _backlightval);
	uint8_t data[1] = {((uint8_t)(_data) | _backlightval)};
	HAL_I2C_Master_Transmit(&hi2c1, (_Addr<<1), data, 1, 100);
}

void lcd_pulseEnable(uint8_t _data){
	lcd_expanderWrite(_data | En);	// En high
	delayMicroseconds(1);		// enable pulse must be >450ns
	
	lcd_expanderWrite(_data & ~En);	// En low
	delayMicroseconds(50);		// lcd_commands need > 37us to settle
} 


// Alias functions

void lcd_cursor_on(){
	lcd_cursor();
}

void lcd_cursor_off(){
	lcd_noCursor();
}

void lcd_blink_on(){
	lcd_blink();
}

void lcd_blink_off(){
	lcd_noBlink();
}


void lcd_setBacklight(uint8_t new_val){
	if(new_val){
		lcd_backlight();		// turn backlight on
	}else{
		lcd_noBacklight();		// turn backlight off
	}
}
