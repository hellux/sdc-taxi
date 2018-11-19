/*
LCD interfacing functions in 4-bit mode

*/

#include<util/delay.h>

#define D7 PORTD7
#define D6 PORTD6
#define D5 PORTD5
#define D4 PORTD4
#define RS PORTD1
#define EN  PORTD0


//Transmit instruction set by already assigned values to databuss and RS
void lcd_send_instruction();

void lcd_send_data();

void lcd_busy_wait();

//initialize lcd in 4 bit mode, see page 13 in datasheet of hd44780 
void lcd_init();

/*
void lcd_write_char(char a){
	
}


*/


