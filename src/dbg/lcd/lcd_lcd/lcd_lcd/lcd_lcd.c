/*
 * lcd_sensor.c
 *
 * Created: 11/15/2018 11:31:35 AM
 *  Author: emiha868
 * 
 * All characters in the rom are ASCII-encoded, 8 bits
 */ 

#define F_CPU 16000000
#include <avr/io.h>
#include "lcd.h"
#include <avr/interrupt.h>

void lcd_send_pulse(){
	PORTD |= (1<<EN);
	_delay_ms(10);
	PORTD &= ~(1<<EN);
	_delay_ms(10);
}
 
//Transmit instruction set by already assigned values to databuss and RS
void lcd_send_instruction(unsigned char instr){
	unsigned char upper = instr;
	upper &= 0xF0;
	upper &= ~(1<<RS);
	
	PORTD = upper;
	lcd_send_pulse();
	
	unsigned char lower = instr;
	lower = (lower << 4);
	lower &= ~(1<<RS);
	
	PORTD = lower;
	lcd_send_pulse();
}

void lcd_set_4bit_interface(){
	PORTD = 0x20;
	PORTD &= ~(1<<RS);
	lcd_send_pulse();
}

void lcd_send_data(unsigned char data){
	unsigned char upper = data;
	upper &= 0xF0;
	upper |= (1<<RS);
	
	PORTD = upper;
	lcd_send_pulse();
	
	unsigned char lower = data;
	lower = (lower << 4);
	lower |= (1<<RS);
	
	PORTD = lower;
	lcd_send_pulse();
}


void lcd_busy_wait(){
	//D5,D4 = 1, RS = 0 R/W = 0
	PORTD = 0x00; // Reset port values from previous
	PORTD |= (1<<D5)|(1<<D4);
	PORTD &= ~(1<<RS);
	lcd_send_pulse();
}

void lcd_reset(){
	//PORTD = 0xFF;
	_delay_ms(20);
	lcd_busy_wait();
	_delay_ms(5);
	lcd_busy_wait();
	_delay_ms(1);
	lcd_busy_wait();
	_delay_ms(1);
}


void lcd_init(){
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
}


void lcd_clear(){
	lcd_send_instruction(0x01);
	_delay_ms(3);
}

/*
void lcd_set_ddram(){
	PORTD = 0x00;
	lcd_send_instruction();
	PORTD = 0x20;
	lcd_send_instruction();
}
*/

void lcd_display_on(){
	//turn on lcd
	lcd_send_instruction(0x0F);
	_delay_ms(300);
}

void lcd_shift_right(){
	lcd_send_instruction(0x14);
}


int main(void)
{
			
	/*All pin on port D are outputs to LCD except PD2 and PD3, inputs from halleffect 
	  PORTD0 (pin 40) and PORTD1(pin 39) are inputs from sensors*/
	DDRD = 0xFF;
	DDRB = 0xFF;
	DDRA = 0xFF;
	//DDRA = 0x00; // inputs from sensors to AD converter
	
    lcd_init();
	_delay_ms(5);
	
	// Display cursor, no blink
	lcd_send_instruction(0x0F);
	
	
	lcd_clear();
	for(int i = 0; i < 4; i++){
		_delay_ms(1000);
		lcd_shift_right();
	}
	
	
	lcd_clear();
	_delay_ms(100);
	
	lcd_send_data(0x41);
	_delay_ms(20);
	lcd_send_data(0x31);
	
	_delay_ms(20);
	lcd_send_data(0x32);
	
	while(1)
    {
		//lcd_send_data(0x41);
		//_delay_ms(20);
		
		/*
		PORTD = 0x42;
		//PORTD |= (1<<RS);
		lcd_send_pulse();
		
		_delay_ms(10);
		
		PORTD = 0x12;
		lcd_send_pulse();
		_delay_ms(1);
		*/
	}
}