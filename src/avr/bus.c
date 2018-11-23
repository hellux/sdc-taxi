#include "bus.h"

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#include "protocol.h"

void spi_init() {
    /* set MISO as output */
    DDRB |= (1<<PB6);

    /* enable SPI Interrupt & SPI */
    SPCR |= (1<<SPIE)|(1<<SPE);

    /* clear data reg */
    SPDR = 0;
}

void spi_tranceive(uint8_t *data, int len) {
    for (int i = 0; i < len; i++) {
        SPDR = data[i];

        while(!(SPSR & (1<<SPIF)));

        data[i] = SPDR;
    }
}

/* accept command from master */
uint8_t spi_accept(uint8_t *data) {
    /* retrieve command */
    cs_t cs;
    spi_tranceive((void*)&cs, 1);
    int cmd = cs_cmd(cs);
    int len = BCMDS[SLAVE][cmd].len;

    /* retrieve data, if any */
    if (BCCS[cmd].write) {
        spi_tranceive(data, len);
    }

    uint8_t ack = ACKS[SLAVE];

    /* if invalid checksum, invert ack and ignore cmd */
    if (!cs_check(cs, data, len)) {
        ack = ~ack;
        cmd = BB_INVALID;
    }

    /* acknowledge to master, return retrieved command */
    spi_tranceive(&ack, sizeof(ack));
    return cmd;
}

/* return data to master, destroy input data */
void spi_return(uint8_t cmd, uint8_t *data, int len) {
    cs_t cs = cs_create(cmd, data, len);
    spi_tranceive(&cs, 1);
    spi_tranceive(data, len);
}
