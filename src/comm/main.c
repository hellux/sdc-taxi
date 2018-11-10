#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "bus.h"
#include "server.h"
#include "img_proc.h"

#define SERVER_PORT 5000

int main(int argc, char* args[]) {
    bool quit = false;

    const char *inet_addr = args[1];
    if (!inet_addr) {
        fprintf(stderr, "error: no IP address specified\n");
        return EXIT_FAILURE;
    }

    bus_t *bus = bus_create();
    if (!bus) return EXIT_FAILURE;
    bus_sens_t sens_data;
    bus_ctrl_t ctrl_data;

    srv_t *srv = srv_create(inet_addr, SERVER_PORT);
    if (!srv) return EXIT_FAILURE;

    while (!quit) {
        srv_listen(srv);

        bus_get_sens(bus, &sens_data);

        bus_receive_sens(bus);
        bus_transmit_ctrl(bus, &ctrl_data);
    }

    bus_destroy(bus);
    srv_destroy(srv);

    return EXIT_SUCCESS;
}
