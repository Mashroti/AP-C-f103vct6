/****************************************/
/*  lcd4bit.cpp							*/
/*  Author: Mohammad Hadi Dashti		*/
/****************************************/

#include "lcd4bit.h"

#define FCLK 72000000



/*void LCD_RS(unsigned char n)
{
	
	if(n)	SetLcdRS;
	else	ClrLcdRS;
}*/

/*void LCD_EN(unsigned char n)
{
	if(n)	SetLcdEN;
	else	ClrLcdEN;
}*/


/*void LCD_RW(unsigned char n)
{
#ifdef DecEnLcdRnW 
	if(n)	SetLcdRW;
	else	ClrLcdRW;
#endif
}*/

void LCD_STROBE(void)
{
	SetLcdEN;
	_delay_us(1);
	ClrLcdEN;
	_delay_us(1);
}

void LCD_out(unsigned char data)
{
	if(data & 0x01) SetLcdD4;
	else ClrLcdD4; 
	if(data & 0x02) SetLcdD5;
	else ClrLcdD5;
	if(data & 0x04) SetLcdD6;
	else ClrLcdD6;
	if(data & 0x08) SetLcdD7;
	else ClrLcdD7;

}


/* write a byte to the LCD in 4 bit mode */
void lcd_write(unsigned char c)
{
	LCD_out(c >> 4);
	LCD_STROBE();
	_delay_us(50);
	LCD_out(c & 0x0F);
	LCD_STROBE();
	_delay_us(50);
}

// Clear and home the LCD
void lcd_clear(void)
{
	ClrLcdRS;
	lcd_write(0x1);
	_delay_ms(2);
}

/* write a string of chars to the LCD */
void lcd_puts(const char* s)
{
	SetLcdRS; /*// write characters*/
	while(*s)
	{
		lcd_write(*s++);
	}
}

/* write one character to the LCD */
void lcd_putch(unsigned char c)
{
	SetLcdRS; /*// write characters*/
	lcd_write(c);
}

//Go to the specified position
void lcd_gotoxy(unsigned char x,unsigned char y)
{
	ClrLcdRS;

	switch(y)
	{
		case 0:
			break;
		case 1:
			x|=0x40;
			break;
		case 2:
			x+=0x14;
			break;
		case 3:
			x+=0x54;
			break;
	}

	x|=0x80;//0b10000000;
	lcd_write(x);
}

void lcd_hide_cursor()
{
	ClrLcdRS;
	lcd_write(0x0C);
}

void lcd_show_cursor(void)
{
	ClrLcdRS;
	lcd_write(0x0F);
}

/* initialise the LCD - put into 4 bit mode */
void lcd_init(void)
{
	ClrLcdRS;
	ClrLcdEN;

	ClrLcdD4;
	ClrLcdD5;
	ClrLcdD6;
	ClrLcdD7;

	ClrLcdRS; // write control bytes
	_delay_ms(100); // power on delay

	LCD_out(0x3); // attention!
	LCD_STROBE();
	_delay_ms(5);
	LCD_out(0x3); // attention!
	LCD_STROBE();
	_delay_ms(5);
	LCD_out(0x3); // attention!
	LCD_STROBE();
	_delay_ms(5);

	LCD_out(0x2); // attention!
	LCD_STROBE();
	_delay_ms(5);

	lcd_write(0x28); // 4 bit mode, 1/16 duty, 5x8 font
	_delay_ms(5);
	lcd_write(0x0F);

	_delay_ms(5);
	lcd_write(0x01);
	_delay_ms(5);
	lcd_clear();
}

void _delay_us(unsigned int Delay)
{
	long long i,k;
	k = (FCLK/21000000)*Delay;
	for(i=0 ; i<k ; i++);
}
