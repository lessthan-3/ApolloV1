#ifndef LCD_H
#define LCD_H


#define F_CPU 8000000UL
#include <avr/io.h>
#include <string.h>
#include <util/delay.h>


//#define F_CPU 8000000UL

#define LCD_DDR DDRD
#define LCD_PORT PORTD
#define LCD_RS_PORT PORTB
#define LCD_RS_DDR DDRB
#define LCD_WR_PORT PORTB
#define LCD_WR_DDR DDRB

#define LCD_RS PB1
#define LCD_WR PB0
#define LCD_E  PD3
#define LCD_D4 PD4
#define LCD_D5 PD5
#define LCD_D6 PD6
#define LCD_D7 PD7

void lcd_init(void);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_print(const char* str);
void lcd_clear(void);

#endif
