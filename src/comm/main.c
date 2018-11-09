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
    srv_t *srv = srv_create("10.121.162.1", 5000);

    bus_sens_t sens_data;
    bus_ctrl_t ctrl_data;

    /* function from c++ */
    ip_process();

    while (!quit) {
        bus_get_sens(bus, &sens_data);
        //printf("rotations: %d\n", sens_data.rotations);

        char str[100];
        //scanf("%s", str);
        if (str[0] == 'q') quit = true;
        ctrl_data.err_vel = str[0];
        bus_receive_sens(bus);
        bus_transmit_ctrl(bus, &ctrl_data);

        srv_listen(srv);
        sleep(1);
    }

    bus_destroy(bus);
    srv_destroy(srv);

    return EXIT_SUCCESS;
}
