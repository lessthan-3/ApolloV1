#include "lcd.h"
#include <util/delay.h>

static void lcd_pulse_enable(void) {
	_delay_us(100);
	LCD_PORT |= (1 << LCD_E);
	_delay_us(50);
	LCD_PORT &= ~(1 << LCD_E);
	_delay_us(50);
}

static void lcd_write_4bits(uint8_t nibble) {
	LCD_PORT &= ~((1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7));

	if (nibble & 0x01) LCD_PORT |= (1 << LCD_D4);
	if (nibble & 0x02) LCD_PORT |= (1 << LCD_D5);
	if (nibble & 0x04) LCD_PORT |= (1 << LCD_D6);
	if (nibble & 0x08) LCD_PORT |= (1 << LCD_D7);

	lcd_pulse_enable();
}

void lcd_command(uint8_t cmd) {
	LCD_RS_PORT &= ~(1 << LCD_RS);  // RS = 0 for command
	lcd_write_4bits(cmd >> 4);  // Upper nibble
	lcd_write_4bits(cmd & 0x0F);      // Lower nibble

	_delay_ms(2);
}

void lcd_data(uint8_t data) {
	LCD_RS_PORT |= (1 << LCD_RS);   // RS = 1 for data
	_delay_us(50);
	lcd_write_4bits(data >> 4);        // Upper nibble
	lcd_write_4bits(data & 0x0F);      // Lower nibble
	
	_delay_us(50);
	LCD_RS_PORT &= ~(1 << LCD_RS);
	
	
	_delay_us(50);
}

void lcd_init(void) {
	// Set pins as output
	LCD_DDR |= (1 << LCD_E) | (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7);
	LCD_RS_DDR |= (1 << LCD_RS);
	LCD_WR_DDR |= (1 << LCD_WR);
	LCD_WR_PORT &= ~(1 << LCD_WR);  // WR low (always write)

	_delay_ms(20);
	LCD_RS_PORT &= ~(1 << LCD_RS);  // RS = 0

	lcd_write_4bits(0x03); // Function set (8-bit) x3
	_delay_ms(5);
	lcd_write_4bits(0x03);
	_delay_us(150);
	lcd_write_4bits(0x03);
	_delay_us(150);

	// Finally, set to 4-bit
	lcd_write_4bits(0x02);
	_delay_us(100);

	// Now full 4-bit commands
	lcd_command(0x28); // 4-bit, 1-line, 5x8
	lcd_command(0x0C); // Display ON, cursor OFF
	lcd_command(0x06); // Entry mode set: increment, no shift
	lcd_command(0x01); // Clear display
	_delay_us(100);
	lcd_command(0x80);
	
	
	
	
	_delay_ms(2);
}

void lcd_clear(void) {
	lcd_command(0x01);
	_delay_ms(2);
}



void lcd_print(const char* str) {
	//lcd_clear();
	lcd_command(0x80);
	_delay_ms(2);
	while (*str) {
		lcd_data((uint8_t)(*str));
		str++;
	}
}
