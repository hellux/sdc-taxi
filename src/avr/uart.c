#include <avr/io.h>
#include <interrupt.h>
#include <util/setbaud.h>


#define F_CPU 16000000



/*INTERRUPT VECTORS
VECTOR			DESCRIPTION

USART0_RX		USART0 Rx Complete
USART0_UDRE		USART0 Data Register Empty
USART0_TX		USART0 Tx Complete
---------------------------------------------

Detailed information about USART/UART starts at page 171
*/


/* The UMSELn bit in USART
Control and Status Register C (UCSRnC) selects between syncrhonous
and asynchronous we want synchronous*/

/*
Double Speed (asynchronous mode only) is controlled by the 2Xn found in the
UCSRnA Register. When using synchronous mode (UMSELn = 1), the Data Direction Register
for the XCKn pin (DDR_XCKn) controls whether the clock source is internal (Master mode) or
external (Slave mode). The XCKn pin is only active when using synchronous mode.
*/


//TODO: 9600 baudrate, calculate
void uart_init(unsigned int baudrate){
	
}



//TODO: Make sure to enable interrupt TX complete and such in register UCSR0B
volatile char ch;
ISR(USART0_TX_vect){		
	UDR=ch; //I assume TX complete means that UDR was sent and is now empty
}


ISR(USART0_TX_vect){
	
}

int main(void)
{
    while(1)
    {
		
	
    }
}