/*
 * lcd_sensor.c
 *
 * Created: 11/15/2018 11:31:35 AM
 *  Author: emiha868
 * 
 * All characters in the rom are ASCII-encoded, 8 bits
 */ 


#include <avr/io.h>
#include "lcd.h"

//Transmit instruction set by already assigned values to databuss and RS
void lcd_send_instruction(){
	PORTD &= ~(1<<RS); // RS=0 when sending instructions
	PORTD |= (1<<EN);
	_delay_ms(1);
	PORTD &= ~(1<<EN);
	_delay_ms(1);
}

void lcd_send_data(){
	PORTD |= (1<<RS); //RS =1 when sending data to ram
	PORTD |= (1<<EN);
	_delay_ms(1);
	PORTD &= ~(1<<EN);
	_delay_ms(1);
}


void lcd_busy_wait(){
	//D5,D4 = 1, RS = 0 R/W = 0
	PORTD &= 0x00; // Reset port values from previous
	PORTD |= (1<<D5)|(1<<D4);
	lcd_send_instruction();
}

//initialize lcd in 4 bit mode, see page 13 in datasheet of hd44780 
void lcd_init(){
	// wait 15ms or more after power on
	_delay_ms(20);
	lcd_busy_wait();
	_delay_ms(10); // wait 4.1ms or more
	lcd_busy_wait();
	_delay_ms(5);//wait 100 micro seconds or more
	lcd_busy_wait();
	
	//STAGE 1 4-BIT FUNCTION SET
	PORTD &= 0x00;
	PORTD |= (1<<D5);
	lcd_send_instruction();
	_delay_ms(2);
	
	//STAGE 2 FUNCTION SET
	PORTD &= 0x00;
	PORTD |= (1<<D5);
	lcd_send_instruction();
	_delay_ms(1);
	
	PORTD &= 0x00;
	PORTD |= (1<<D7)|(1<<D6);
	lcd_send_instruction();
	_delay_ms(1);
	
	//STAGE 3 DISPLAY OFF
	PORTD &= 0x00;
	//SEND just zeroes for some reason
	lcd_send_instruction();
	_delay_ms(1);
	
	PORTD |= (1<<D4);
	lcd_send_instruction();
	_delay_ms(2);
	
	//STAGE 4 DISPLAY CLEAR
	PORTD &= 0x00;
	lcd_send_instruction();
	_delay_ms(2);
	PORTD |= (1<<D4);
	lcd_send_instruction();
	_delay_ms(4);
	
	/*
	STAGE 5 ENTRY MODE SET
	SET I/D=1 CURSOR TO MOVE TO RIGHT AND DDRAM INCREASED BY 1
	SET SH=0 SHIFTING OF ENTIRE DISPLAY IS OFF
	*/
	PORTD &=0x00;
	lcd_send_instruction();
	_delay_ms(2);
	//D6=1 I/D=1 SH=0
	PORTD |= (1<<D6)|(1<<D5);
	lcd_send_instruction();
	_delay_ms(2);
	
	//turn on lcd
	PORTD = 0x00;
	lcd_send_instruction();
	PORTD = 0xF0;
	lcd_send_instruction();
	_delay_ms(3000);
}


int main(void)
{
	_delay_ms(40);
	
	uint8_t counter;
	
	counter = 0;
	
	/*All pin on port D are outputs to LCD except PD2 and PD3, inputs from halleffect 
	  PORTA0 (pin 40) and PORTA1(pin 39) are inputs from sensors*/
	DDRD = 0xFF;
	DDRB = 0xFF;
	//DDRA = 0x00; // inputs from sensors to AD converter
	
    lcd_init();
	
	//send letter A
	PORTD = 0x00;
	PORTD |=(1<<RS);
	lcd_send_data();
	_delay_ms(1);
	PORTD=0x10;
	lcd_send_data();
	_delay_ms(1);
	
	while(1)
    {
		counter++;
		PORTB = counter;
		
		
	
		
		
		


    }
}