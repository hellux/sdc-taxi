#include <avr/io.h>

void spi_init();
uint8_t spi_accept(uint8_t *data, uint8_t ack);
void spi_return(uint8_t cmd, uint8_t *data, int len);
