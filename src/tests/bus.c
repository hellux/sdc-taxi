#include <assert.h>
#include <string.h>
#include <stdio.h>

#include "comm/bus.h"
#include "spi/protocol.h"

int main(void) {
    bus_t *bus = bus_create(500000);

    char *msg1 = "hej hej";
    struct bus_cmd bc1 = {
        .cmd = 5,
        .write = true,
        .slave = 0,
        .len = sizeof(msg1),
    };
    
    bus_destroy(bus);

    return 0;
}
