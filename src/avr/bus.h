#include <avr/io.h>

#define SPI_INSIDE_ISR 1
#define SPI_OUTSIDE_ISR 0

#define SPI_ENABLE_INT 1
#define SPI_DISABLE_INT 0

void spi_init(int interrupt);
uint8_t spi_accept(uint8_t *data, int interrupt);
void spi_return(uint8_t cmd, uint8_t *data, int len);
