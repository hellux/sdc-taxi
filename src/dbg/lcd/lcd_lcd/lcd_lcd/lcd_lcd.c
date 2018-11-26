/*
 * lcd_sensor.c
 *
 * Created: 11/15/2018 11:31:35 AM
 *  Author: emiha868
 * 
 * All characters in the rom are ASCII-encoded, 8 bits
 *
 * LCD interfacing functions in 4-bit mode
 *
 */



#define D7 PORTA7
#define D6 PORTA6
#define D5 PORTA5
#define D4 PORTA4
#define RS PORTA3
#define EN PORTA2

#define F_CPU 16000000
#include<util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#define ADC_PRESCALER_128 0x07
#define CNV_FRONT_MUL 17391
#define CNV_FRONT_EXP 1.071

#define CNV_RIGHT_MUL 2680
#define CNV_RIGHT_EXP 1.018

#define CHN_SENS_FRONT 0
#define CHN_SENS_RIGHT 1

#define WHEEL_DIAM 0.08
#define WHEEL_SENS_FREQ 5
#define PI 3.14


void lcd_send_pulse(){
	PORTA |= (1<<EN);
	_delay_ms(2);
	PORTA &= ~(1<<EN);
	_delay_ms(2);
}
 
//Transmit instruction set by already assigned values to databuss and RS
void lcd_send_instruction(unsigned char instr){
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
void lcd_set_4bit_interface(){
	PORTA = 0x20;
	PORTA &= ~(1<<RS);
	lcd_send_pulse();
}

//Write a character to display
void lcd_send_data(unsigned char data){
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

//send a string to display, 2x16, hardcoded scrolling.
void lcd_send_string(char *string) {
	
	int len = strlen(string);
	
	for (int i = 0; i < len; i++){
		
		//RADBRYTNING OM SISTA BOKSTAV I RAD
		if( i ==0x0F || i == 0x1F ){
			//2x16, 0F är sista bokstav i rad 1 hoppa till 0x40, first letter second row
			// 1F är sista bosktav i rad 2, hoppa tillbaka hem
			lcd_send_data(string[i]);
			char c = (i == 0x0F) ? 0x40 : 0x00;
			c |= ( 1 << D7 );
			lcd_send_instruction(c);
			
		}else{
			lcd_send_data(string[i]);
		}
	
	}
	
}

//part of init
void lcd_busy_wait(){
	//D5,D4 = 1, RS = 0 R/W = 0
	PORTA = 0x00; // Reset port values from previous
	PORTA |= (1<<D5)|(1<<D4);
	PORTA &= ~(1<<RS);
	lcd_send_pulse();
}

//part of init
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

//initialize 4 bit mode
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
	
	// Display cursor, no blink
	lcd_send_instruction(0x0F);
}


//instruction for clear
void lcd_clear(){
	lcd_send_instruction(0x01);
	_delay_ms(3);
}


//instruction for display on
void lcd_display_on(){
	//turn on lcd
	lcd_send_instruction(0x0F);
	_delay_ms(300);
}

//instruction shift cursor to right
void lcd_shift_right(){
	lcd_send_instruction(0x14);
}

void adc_init(void) {
	//mux init
	ADMUX = (1 << REFS0);

	//ADC enable
	ADCSRA |= (1 << ADEN);

	/* F_ADC = F_CPU/prescaler */
	ADCSRA |= ADC_PRESCALER_128;

	//ADC interrupt enabled
	ADCSRA |= (1 << ADIE);
}

uint16_t adc_read(uint8_t channel) {
	//Set multiplexer for given channel (0-7) (front or back sensor)
	ADMUX = (ADMUX & 0xF8) | channel; //ADMUX &= 0xE0; //Clear the older channel that was read?

	// start single convertion
	ADCSRA |= (1 << ADSC);

	// wait until it is ready (=0)
	while (ADCSRA & (1 << ADSC));

	return (ADC); //ADCL-ADCH
}



int main(void)
{
			
	DDRA = 0xFA;
	
    lcd_init();
	_delay_ms(5);
	lcd_clear();
	

	
	lcd_send_string( "Initializing m8" );
	_delay_ms(2000);
	lcd_clear();
	
	while(1)
	{
		uint16_t adc_right = adc_read(CHN_SENS_RIGHT);
		float dist = CNV_FRONT_EXP * pow(adc_right, -CNV_RIGHT_EXP);
		_delay_ms(10);
		char* buf[32];
		sprintf(buf,"%f", dist);
		lcd_send_string(strcat("RIGHT: ", buf));
			
	}
}