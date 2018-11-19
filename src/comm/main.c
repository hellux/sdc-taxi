#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include <pthread.h>

#include "bus.h"
#include "server.h"
#include "ip/img_proc.h"

#include "../spi/protocol.h"

#define SERVER_PORT_START 9000
#define SERVER_PORT_END 9100

#define SLAVE_SENS 7
#define SLAVE_CTRL 8

/* bus commands for ctrl */
struct bus_cmd BC_SPEED =    {BCB_SPEED,    SLAVE_CTRL, sizeof(ctrl_val_t)};
struct bus_cmd BC_SPEED_KD = {BCB_SPEED_KD, SLAVE_CTRL, sizeof(ctrl_const_t)};
struct bus_cmd BC_SPEED_KP = {BCB_SPEED_KP, SLAVE_CTRL, sizeof(ctrl_const_t)};
struct bus_cmd BC_TURN =     {BCB_TURN,     SLAVE_CTRL, sizeof(ctrl_val_t)};
struct bus_cmd BC_TURN_KD =  {BCB_TURN_KD,  SLAVE_CTRL, sizeof(ctrl_const_t)};
struct bus_cmd BC_TURN_KP =  {BCB_TURN_KP,  SLAVE_CTRL, sizeof(ctrl_const_t)};
struct bus_cmd BC_RST_CTRL = {BCB_RESET,    SLAVE_CTRL, 0 };

/* bus commands for sens */
const struct bus_cmd BC_GET_SENS = {BCB_GET_SENS, SLAVE_SENS,
    sizeof(struct sens_data_frame)};
const struct bus_cmd BC_RST_SENS = {BCB_RESET,    SLAVE_CTRL, 0};

/* data from sensors stored on pi */
struct data_sensors {
    sens_dist_t dist_front;
    sens_dist_t dist_right;
    unsigned rotations;
    
    pthread_mutex_t lock;
};

struct data_mission {
    bool active;

    /* TODO commands */
    int cmds_completed;
    int cmds_remaining;

    pthread_mutex_t lock;
};

struct data_rc {
    double err_vel;
    double err_rot;

    pthread_mutex_t lock;
};

/* server commands, called by server on remote command */

bool sc_get_sens(struct srv_cmd_args *a) {
    struct data_sensors *sens_data = (struct data_sensors*)a->data1;

    /* read data */
    sens_dist_t df, dr;
    unsigned rotations;
    pthread_mutex_lock(&sens_data->lock);
    df = sens_data->dist_front;
    dr = sens_data->dist_right;
    rotations = sens_data->rotations;
    pthread_mutex_unlock(&sens_data->lock);

    /* create string */
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';
    rsp = str_append(rsp, &buf_size, "df=%d ", df);
    rsp = str_append(rsp, &buf_size, "dr=%d ", dr);
    rsp = str_append(rsp, &buf_size, "rotations=%d", rotations);

    a->resp = rsp;
    return true;
}

bool sc_get_mission(struct srv_cmd_args *a) {
    struct data_mission *miss_data = (struct data_mission*)a->data1;

    /* read data */
    int cmp, rem;
    pthread_mutex_lock(&miss_data->lock);
    cmp = miss_data->cmds_completed;
    rem = miss_data->cmds_remaining;
    pthread_mutex_unlock(&miss_data->lock);

    /* create string */
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';
    rsp = str_append(rsp, &buf_size, "completed=%d ", cmp);
    rsp = str_append(rsp, &buf_size, "remaining=%d", rem);

    a->resp = rsp;
    return true;
}

bool sc_set_mission(struct srv_cmd_args *a) {
    return true;
}

bool sc_set_state(struct srv_cmd_args *a) {
    return true;
}

bool sc_bus_send_float(struct srv_cmd_args *a) {
    int success = false;
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';
    
    char* float_str = a->args[1];
    char *endptr;
    float value = strtof(float_str, &endptr);

    if (endptr > float_str) {
        struct bus_cmd *bc = (struct bus_cmd*)a->data1;
        bus_t *bus = (bus_t*)a->data2;
        bus_transmit_schedule(bus, bc, (unsigned char*)&value, NULL, NULL);

        success = true;
        rsp = str_append(rsp, &buf_size, "sending value %f", value);
    } else {
        rsp = str_append(rsp, &buf_size,
                         "invalid argument -- \"%s\"", float_str);
    }

    a->resp = rsp;
    return success;
}

/* bus signal handlers, called by bus thread when transmission finished */

/* write received values to struct reachable from main thread */
void bsh_sens_recv(void *received, void *data) {
    struct sens_data_frame *frame = (struct sens_data_frame*)received;
    struct data_sensors *sens_data = (struct data_sensors*)data;
    
    pthread_mutex_lock(&sens_data->lock);
    sens_data->dist_front = frame->dist_front;
    sens_data->dist_right = frame->dist_right;
    sens_data->rotations += frame->rotations;
    pthread_mutex_unlock(&sens_data->lock);
}

int main(int argc, char* args[]) {
    bool quit = false;

    const char *inet_addr = args[1];
    if (!inet_addr) {
        fprintf(stderr, "error: no IP address specified\n");
        return EXIT_FAILURE;
    }

    struct data_sensors sens_data = {0};
    struct data_mission miss_data = {0};
    struct data_rc rc_data = {0};
    pthread_mutex_init(&sens_data.lock, 0);
    pthread_mutex_init(&miss_data.lock, 0);
    pthread_mutex_init(&rc_data.lock, 0);

    bus_t *bus = bus_create(F_SPI);
    if (!bus) return EXIT_FAILURE;

    struct srv_cmd cmds[] = {
        {"shutdown",        0, &quit,        NULL, NULL},     /* TODO enable restart from remote */
        {"get_sensor_data", 0, &sens_data,   NULL, *sc_get_sens},
        {"get_mission",     0, &miss_data,   NULL, *sc_get_mission},
        {"set_mission",     1, NULL,         NULL, *sc_set_mission},  /* TODO */
        {"set_state",       1, NULL,         NULL, *sc_set_state},    /* TODO */
        {"set_speed_delta", 1, NULL,         bus,  NULL},             /* TODO */
        {"set_speed_kp",    1, &BC_SPEED_KP, bus,  *sc_bus_send_float},
        {"set_speed_kd",    1, &BC_SPEED_KD, bus,  *sc_bus_send_float},
        {"set_turn_delta",  1, NULL,         NULL, NULL},             /* TODO */
        {"set_turn_kp",     1, &BC_TURN_KP,  bus,  *sc_bus_send_float},
        {"set_turn_kd",     1, &BC_TURN_KD,  bus,  *sc_bus_send_float},
    };
    int cmdc = sizeof(cmds)/sizeof(*cmds);
    srv_t *srv = srv_create(inet_addr, SERVER_PORT_START, SERVER_PORT_END,
                            cmds, cmdc);
    if (!srv) return EXIT_FAILURE;

    /* remote ctrl values */

    char input[100];
    while (!quit) {
        double err_vel;
        double err_rot;

        bus_receive_schedule(bus, &BC_GET_SENS, bsh_sens_recv, &sens_data);

        pthread_mutex_lock(&miss_data.lock);
        if (miss_data.active) {
            pthread_mutex_unlock(&miss_data.lock);
            /* TODO img proc + mission */
            err_vel = 0;
            err_rot = 0;
        } else {
            pthread_mutex_unlock(&miss_data.lock);

            pthread_mutex_lock(&rc_data.lock);
            err_vel = rc_data.err_vel;
            err_rot = rc_data.err_rot;
            pthread_mutex_unlock(&rc_data.lock);
        }

        /* pause loop, enable exit */
        scanf("%s", input);
        if (input[0] == 'q')
            quit = true;

        bus_transmit_schedule(bus, &BC_SPEED, (void*)&err_vel, NULL, NULL);
        bus_transmit_schedule(bus, &BC_TURN, (void*)&err_rot, NULL, NULL);
    }

    srv_destroy(srv);
    bus_destroy(bus);

    return EXIT_SUCCESS;
}
