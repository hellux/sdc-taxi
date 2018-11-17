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

bool cmd_check(int argc, char **args, char **rsp_dst, void *d1, void *d2) {
    const int BUF_START = 30;
    int bufs = BUF_START;
    char *rsp = malloc(bufs);
    char *str_pos = rsp;
    str_pos += sprintf(str_pos, "argc: %d, args: ", argc);

    for (int i = 0; i < argc; i++) {
        int max_size = bufs-(str_pos-rsp);
        int len = snprintf(str_pos, max_size, "\"%s\", ", args[i]);
        if (len < max_size) {
            str_pos += len;
        } else {
            int total_length = str_pos-rsp;
            bufs *= 2;
            rsp = realloc(rsp, bufs);
            str_pos = rsp+total_length;
            i--;
        }
    }
    /* erase last comma and space */
    str_pos -= 2;
    str_pos += sprintf(str_pos, ".");
    *rsp_dst = rsp;
    return true;
}

bool cmd_help(int argc, char **args, char **rsp_dst, void *d1, void *d2) {
    char *response = malloc(1024);
    struct srv_cmd *cmds = (struct srv_cmd*)d1;
    char *str_pos = response;
    int cmdc = *(int*)d2;
    str_pos += sprintf(str_pos, "available commands: ");
    for (int i = 0; i < cmdc; i++) {
        str_pos += sprintf(str_pos, "%s, ", cmds[i].name);
    }
    /* erase last comma and space */
    str_pos -= 2;
    str_pos += sprintf(str_pos, ".");

    *rsp_dst = response;
    return true;
}

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

    int cmdc;
    struct srv_cmd cmds[] = {
        {"help",            0, cmds, &cmdc, *cmd_help},
        {"check",           0, NULL, NULL, *cmd_check},
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
    cmdc = sizeof(cmds)/sizeof(*cmds);
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
