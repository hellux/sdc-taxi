#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "../comm/bus.h"

int main() {
    bool quit = false;
    bus_t *bus = bus_create();

    bus_sens_t sens_data;
    bus_ctrl_t ctrl_data;

    while (!quit) {
        bus_get_sens(bus, &sens_data);
        printf("rotations: %d\n", sens_data.rotations);

        char str[100];
        scanf("%s", str);
        if (str[0] == 'q') quit = true;
        ctrl_data.err_vel = str[0];

        bus_receive_sens(bus);
        bus_transmit_ctrl(bus, &ctrl_data);
    }

    bus_destroy(bus);

    return EXIT_SUCCESS;
}
