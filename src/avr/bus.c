#include "bus.h"

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#include "protocol.h"

void spi_init(int enable_interrupt) {
    /* set MISO as output */
    DDRB |= (1<<PB6);

    /* enable SPI Interrupt & SPI */
    SPCR |= (enable_interrupt<<SPIE)|(1<<SPE);

    /* clear data reg */
    SPDR = 0;
}

void spi_tranceive(uint8_t *data, int len) {
    for (int i = 0; i < len; i++) {
        SPDR = data[i];

        while (!(SPSR & (1<<SPIF)));

        data[i] = SPDR;
    }
}

/* accept command from master */
uint8_t spi_accept(uint8_t *data, int interrupt, int slave) {
    /* retrieve command and checksum */
    cs_t cs;
    if (interrupt == SPI_INSIDE_ISR) {
        cs = SPDR; /* already stored in spdr */
    } else {
        /* wait until retrieved */
        spi_tranceive((uint8_t*)&cs, 1);
    }
    int cmd = cs_cmd(cs);
    const struct bus_cmd *bc = &BCMDS[slave][cmd];

    /* synchronize if invalid cmd */
    if (!bc->valid) {
        return BB_INVALID;
    }

    uint8_t ack = ACKS[slave];
    /* retrieve data, if any */
    if (bc->write) {
        spi_tranceive(data, bc->len);

        /* if invalid checksum, ignore to resync */
        if (!cs_check(cs, data, bc->len)) {
            return BB_INVALID;
        }

        /* acknowledge to master, return retrieved command */
        spi_tranceive(&ack, sizeof(ack));
    } else {
        spi_tranceive(&ack, sizeof(ack));
    }
    return cmd;
}

/* return data to master, destroy input data */
void spi_return(uint8_t cmd, uint8_t *data, int len) {
    cs_t cs = cs_create(cmd, data, len);
    spi_tranceive(&cs, 1);
    spi_tranceive(data, len);
}
