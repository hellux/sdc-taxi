#include "jtag.h"

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

void init_jtagport(){
    //Set TDO to output
    DDRC |= (1<<PC4);
}
