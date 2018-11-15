/*
 * lcd_sensor.c
 *
 * Created: 11/15/2018 11:31:35 AM
 *  Author: emiha868
 */ 


#include <avr/io.h>
#include <delay.h>


int main(void)
{
	
	uint8_t counter;
	counter = 0;
	DDRB = 0xFF; 
    while(1)
    {
        //TODO:: Please write your application code
		counter++;
		PORTA=counter;
    }
	
}