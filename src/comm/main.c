#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "bus.h"
#include "server.h"
#include "ip/img_proc.h"

#define SERVER_PORT_START 9000
#define SERVER_PORT_END 9100

#define F_SPI 4000000

struct srv_set_values {
    char *mission;
    double speed_err;
    double speed_kp;
    double speed_kd;
    double turn_err;
    double turn_kp;
    double turn_kd;
    bool state;
};

/* server commands */

bool cmd_get_sens(int argc, char **args, char **rsp_dst, void *d1, void *d2) {
    return true;
}

bool cmd_get_mission(int argc, char **args, char **rsp_dst, void *d1, void *d2) {
    return true;
}

bool cmd_set_mission(int argc, char **args, char **rsp_dst, void *d1, void *d2) {
    return true;
}

bool cmd_set_bool(int argc, char **args, char **rsp_dst, void *d1, void *d2) {
    return true;
}

bool cmd_set_float(int argc, char **args, char **rsp_dst, void *d1, void *d2) {
    return true;
}


int main(int argc, char* args[]) {
    bool quit = false;

    const char *inet_addr = args[1];
    if (!inet_addr) {
        fprintf(stderr, "error: no IP address specified\n");
        return EXIT_FAILURE;
    }

    struct reg_consts;
    struct srv_cmd cmds[] = {
        {"get_sensor_data", 0, NULL, NULL, *cmd_get_sens},
        {"get_mission",     0, NULL, NULL, *cmd_get_mission},
        {"set_mission",     0, NULL, NULL, *cmd_set_mission},
        {"set_state",       0, NULL, NULL, *cmd_set_bool},
        {"set_speed_delta", 0, NULL, NULL, *cmd_set_float},
        {"set_speed_kp",    0, NULL, NULL, *cmd_set_float},
        {"set_speed_kd",    0, NULL, NULL, *cmd_set_float},
        {"set_turn_delta",  0, NULL, NULL, *cmd_set_float},
        {"set_turn_kp",     0, NULL, NULL, *cmd_set_float},
        {"set_turn_kd",     0, NULL, NULL, *cmd_set_float},
    };
    int cmdc = sizeof(cmds)/sizeof(*cmds);
    srv_t *srv = srv_create(inet_addr, SERVER_PORT_START, SERVER_PORT_END,
                            cmds, cmdc);
    if (!srv) return EXIT_FAILURE;

    bus_t *bus = bus_create(F_SPI);
    if (!bus) return EXIT_FAILURE;
    bus_sens_t sens_data;
    bus_ctrl_t ctrl_data;

    char input[100];
    while (!quit) {
        scanf("%s", input);
        if (input[0] == 'q')
            quit = true;
        bus_get_sens(bus, &sens_data);
        bus_receive_sens(bus);
        bus_transmit_ctrl(bus, &ctrl_data);
    }

    bus_destroy(bus);
    srv_destroy(srv);

    return EXIT_SUCCESS;
}
