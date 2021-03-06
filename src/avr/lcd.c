#include "lcd.h"

#define D7 PORTA7
#define D6 PORTA6
#define D5 PORTA5
#define D4 PORTA4
#define RS PORTA3
#define EN PORTA2

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

void lcd_send_pulse(void) {
	PORTA |= (1<<EN);
	_delay_ms(2);
	PORTA &= ~(1<<EN);
	_delay_ms(2);
}
 
//Transmit instruction set by already assigned values to databuss and RS
void lcd_send_instruction(unsigned char instr) {
	unsigned char upper = instr;
	upper &= 0xF0;
	upper &= ~(1<<RS);
	
	PORTA = upper;
	lcd_send_pulse();
	
	unsigned char lower = instr;
	lower = (lower << 4);
	lower &= ~(1<<RS);
	
	PORTA = lower;
	lcd_send_pulse();
}

//part of init
void lcd_set_4bit_interface(void) {
	PORTA = 0x20;
	PORTA &= ~(1<<RS);
	lcd_send_pulse();
}

//Write a character to display
void lcd_send_data(unsigned char data) {
	unsigned char upper = data;
	upper &= 0xF0;
	upper |= (1<<RS);
	
	PORTA = upper;
	lcd_send_pulse();
	
	unsigned char lower = data;
	lower = (lower << 4);
	lower |= (1<<RS);
	
	PORTA = lower;
	lcd_send_pulse();
}

//part of init
void lcd_busy_wait(void) {
	//D5,D4 = 1, RS = 0 R/W = 0
	PORTA = 0x00; // Reset port values from previous
	PORTA |= (1<<D5)|(1<<D4);
	PORTA &= ~(1<<RS);
	lcd_send_pulse();
}

//part of init
void lcd_reset(void) {
	//PORTD = 0xFF;
	_delay_ms(20);
	lcd_busy_wait();
	_delay_ms(5);
	lcd_busy_wait();
	_delay_ms(1);
	lcd_busy_wait();
	_delay_ms(1);
}

//initialize 4 bit mode
void lcd_init(void) {
	lcd_reset();

	//STAGE 1
	lcd_set_4bit_interface();
	
	//STAGE 2 4 bit mode 2 line 5x8 font
	lcd_send_instruction(0x28);
	
	//STAGE 3 DISPLAY OFF
	lcd_send_instruction(0x80);
	
	//STAGE 4 DISPLAY CLEAR
	lcd_send_instruction(0x01);
	
	//STAGE 5 I/D = 1 D6 = 1 S = 0
	lcd_send_instruction(0x06);
	
	// Display cursor, no blink
	lcd_send_instruction(0x0F);
}

//instruction for clear
void lcd_clear(void) {
	lcd_send_instruction(0x01); //D0 = 1
	_delay_ms(3);
}

//instruction for display on
void lcd_display_on(void) {
	//turn on lcd
	lcd_send_instruction(0x0F);
	_delay_ms(300);
}

//instruction shift cursor to right
void lcd_shift_right(void) {
	lcd_send_instruction(0x14);
}

void lcd_set_ddram(char c) {
	//MOVES CURSOR TO ADDRESS c
	c |= ( 1 << D7 );  // SET DDRAM ADDRESS INSTRUCTION
	lcd_send_instruction(c);	
}

void lcd_return_cursor_home(void) {
	/*
	Set ddram address to 0x00 from AC,
	Return cursor to its original position if shifted.
	The contents of ddram are not changed.
	*/
	lcd_send_instruction(0x02); //D1 =1
	_delay_ms(2);
}

//send a string to display, 2x16, hardcoded scrolling.
void lcd_send_string(char *string) {
	int len = strlen(string);
	char row = 0;
	
	for (int i = 0; i < len; i++) {
		int mask = i;
		//RADBRYTNING OM SISTA BOKSTAV I RAD
		if ((mask &= 0x0F) && mask == 0x0F) {
			// Var 16:de bokstav, hoppa till n�sta rad.
			row = ~(row);
			lcd_send_data(string[i]);
			char c = (row == 0) ? ROW1ADR : ROW2ADR;
			lcd_set_ddram(c);
		} else {
			lcd_send_data(string[i]);
	    }
	}
}
