/*
LCD interfacing functions in 4-bit mode

*/



#define D7 PORTA7
#define D6 PORTA6
#define D5 PORTA5
#define D4 PORTA4
#define RS PORTA3
#define EN PORTA2


//Transmit instruction set by already assigned values to databuss and RS
void lcd_send_instruction(unsigned char instr);

void lcd_send_data(unsigned char data);

void lcd_busy_wait();

//initialize lcd in 4 bit mode, see page 13 in datasheet of hd44780 
void lcd_init();

/*
void lcd_write_char(char a){
	
}


*/


