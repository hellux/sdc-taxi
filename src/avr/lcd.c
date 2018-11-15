#include "lcd.h"

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

void init_lcdports(){
    //Set ouput ports to LCD
    DDRA |= (1<<PA4)|(1<<PA5)|(1<<PA6)|(1<<PA7);
    DDRB |= (1<<PB0)|(1<<PB1);
}