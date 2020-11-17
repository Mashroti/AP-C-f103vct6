/****************************************/
/*  lcd4bit.h							*/
/*  Author: Mohammad Hadi Dashti		*/
/****************************************/


//---------------------------------------------------------------------
//---------------------------------------------------------------------

#ifndef Lcd4Bit_h
#define Lcd4Bit_h
#ifdef __cplusplus
extern "C" {
	#endif
#include "main.h"
	//#include "stm32f1xx_hal.h"

#define _delay_ms(x)	HAL_Delay(x)
	
/************************************************
	LCD CONNECTIONS
*************************************************/
#define lcd_puts_XY(x,y,msg) {\
 lcd_gotoxy(x,y);\
 lcd_puts(msg);\
}

#define SetLcdRS				LCD_RS_GPIO_Port->BSRR = LCD_RS_Pin
#define ClrLcdRS				LCD_RS_GPIO_Port->BSRR = (uint32_t)LCD_RS_Pin << 16u

#define SetLcdEN				LCD_E_GPIO_Port->BSRR = LCD_E_Pin
#define ClrLcdEN				LCD_E_GPIO_Port->BSRR = (uint32_t)LCD_E_Pin << 16u


#define SetLcdD4				LCD_D4_GPIO_Port->BSRR = LCD_D4_Pin
#define ClrLcdD4				LCD_D4_GPIO_Port->BSRR = (uint32_t)LCD_D4_Pin << 16u

#define SetLcdD5				LCD_D5_GPIO_Port->BSRR = LCD_D5_Pin
#define ClrLcdD5				LCD_D5_GPIO_Port->BSRR = (uint32_t)LCD_D5_Pin << 16u

#define SetLcdD6				LCD_D6_GPIO_Port->BSRR = LCD_D6_Pin
#define ClrLcdD6				LCD_D6_GPIO_Port->BSRR = (uint32_t)LCD_D6_Pin << 16u

#define SetLcdD7				LCD_D7_GPIO_Port->BSRR = LCD_D7_Pin
#define ClrLcdD7				LCD_D7_GPIO_Port->BSRR = (uint32_t)LCD_D7_Pin << 16u


/***************************************************
			F U N C T I O N S
****************************************************/

//void LCD_RS(unsigned char n);
//void LCD_EN(unsigned char n);
//void LCD_RW(unsigned char n);
void LCD_STROBE(void);
void LCD_out(unsigned char data);
void lcd_write(unsigned char c);
void lcd_clear(void);
void lcd_puts(const char * s);
void lcd_putch(unsigned char c);
void lcd_gotoxy(unsigned char x,unsigned char y);
void lcd_hide_cursor(void);
void lcd_show_cursor(void);
void lcd_init(void);
void _delay_us(unsigned int);
#ifdef __cplusplus
}
#endif
#endif	/*	Lcd4Bit_h	*/

