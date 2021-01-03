//YWROBOT
#ifndef LiquidCrystal_I2C_h
#define LiquidCrystal_I2C_h

#include <stdint.h>

#define FCLK	72000000
#define hi2c	hi2c1


// commands
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

#define En 0B00000100  // Enable bit
#define Rw 0B00000010  // Read/Write bit
#define Rs 0B00000001  // Register select bit


  void lcd_begin(uint8_t cols, uint8_t rows);
  void lcd_clear();
  void lcd_home();
  void lcd_noDisplay();
  void lcd_display();
  void lcd_noBlink();
  void lcd_blink();
  void lcd_noCursor();
  void lcd_cursor();
  void lcd_scrollDisplayLeft();
  void lcd_scrollDisplayRight();
  void lcd_printLeft();
  void lcd_printRight();
  void lcd_leftToRight();
  void lcd_rightToLeft();
  void lcd_shiftIncrement();
  void lcd_shiftDecrement();
  void lcd_noBacklight();
  void lcd_backlight();
  void lcd_autoscroll();
  void lcd_noAutoscroll();
  void lcd_createChar(uint8_t, uint8_t[]);
  void lcd_puts(const char* s);
  void lcd_putch(char character);
  void lcd_puts_XY(uint8_t x,uint8_t y, char* s);
  // Example: 	const char bell[8] PROGMEM = {B00100,B01110,B01110,B01110,B11111,B00000,B00100,B00000};
  
  void lcd_setCursor(uint8_t, uint8_t);

  uint8_t lcd_write(uint8_t);

  void lcd_command(uint8_t);
  void lcd_init(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows);

////compatibility API function aliases
void lcd_blink_on();						// alias for blink()
void lcd_blink_off();       					// alias for noBlink()
void lcd_cursor_on();      	 					// alias for cursor()
void lcd_cursor_off();      					// alias for noCursor()
void lcd_setBacklight(uint8_t new_val);				// alias for backlight() and nobacklight()


  void lcd_init_priv();
  void lcd_send(uint8_t, uint8_t);
  void lcd_write4bits(uint8_t);
  void lcd_expanderWrite(uint8_t);
  void lcd_pulseEnable(uint8_t);
  uint8_t _Addr;
  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;
  uint8_t _numlines;
  uint8_t _cols;
  uint8_t _rows;
  uint8_t _backlightval;

#endif
