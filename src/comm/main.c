#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include <pthread.h>

#include "bus.h"
#include "server.h"
#include "img_proc.h"

int main(int argc, char* args[]) {
    bool quit = false;
    bus_t *bus = bus_create();
    srv_t *srv = srv_create("127.0.0.1", 5000);

    bus_sens_t sens_data;
    bus_ctrl_t ctrl_data;

    /* function from c++ */
    ip_process();

    while (!quit) {
        bus_get_sens(bus, &sens_data);

        bus_receive_sens(bus);
        bus_transmit_ctrl(bus, &ctrl_data);

        srv_listen(srv);
    }

    bus_destroy(bus);
    srv_destroy(srv);

    return EXIT_SUCCESS;
}
