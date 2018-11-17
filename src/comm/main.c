#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include <pthread.h>

#include "bus.h"
#include "server.h"
#include "ip/img_proc.h"

#define SERVER_PORT_START 9000
#define SERVER_PORT_END 9100

#define F_SPI 4000000

struct sensor_data {
    uint8_t dist_front;
    uint8_t dist_right;
    unsigned rotations;
    
    pthread_mutex_t lock;
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

void hlr_sens_recv(unsigned char* received, void *data) {
    char *msg = (char*)data;
    received[9] = '\0';
    printf("%s, received: %s\n", msg, received);
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

    char *msg = "hej hej";
    bus_receive_schedule(bus, 0, 7, 10, hlr_sens_recv, msg);
    unsigned char dst[10];
    bus_receive(bus, 0, 7, dst, 10);
    printf("received: %s\n", dst);

    char input[100];
    while (!quit) {
        /* pause loop, enable exit */
        scanf("%s", input);
        if (input[0] == 'q')
            quit = true;

        /* TODO double err = ip_process() */
    }

    bus_destroy(bus);
    srv_destroy(srv);

    return EXIT_SUCCESS;
}
