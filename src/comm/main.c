#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

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
struct bus_cmd BCC_VEL_VAL = {BCBC_VEL_VAL, SLAVE_CTRL, sizeof(ctrl_val_t)};
struct bus_cmd BCC_VEL_ERR = {BCBC_VEL_ERR, SLAVE_CTRL, sizeof(ctrl_val_t)};
struct bus_cmd BCC_VEL_KP  = {BCBC_VEL_KP,  SLAVE_CTRL, sizeof(ctrl_val_t)};
struct bus_cmd BCC_VEL_KD  = {BCBC_VEL_KD,  SLAVE_CTRL, sizeof(ctrl_val_t)};
struct bus_cmd BCC_ROT_VAL = {BCBC_ROT_VAL, SLAVE_CTRL, sizeof(ctrl_val_t)};
struct bus_cmd BCC_ROT_ERR = {BCBC_ROT_ERR, SLAVE_CTRL, sizeof(ctrl_val_t)};
struct bus_cmd BCC_ROT_KP  = {BCBC_ROT_KP,  SLAVE_CTRL, sizeof(ctrl_val_t)};
struct bus_cmd BCC_ROT_KD  = {BCBC_ROT_KD,  SLAVE_CTRL, sizeof(ctrl_val_t)};
struct bus_cmd BCC_RST     = {BCBC_RST,     SLAVE_CTRL, 0};
struct bus_cmd BCC_SYN     = {BCBC_RST,     SLAVE_CTRL, 1};

/* bus commands for sens */
struct bus_cmd BCS_GET = {BCBS_GET, SLAVE_SENS, sizeof(struct sens_data)};
struct bus_cmd BCS_RST = {BCBS_RST, SLAVE_CTRL, 0};

/* data from sensors stored on pi */
struct data_sensors {
    struct sens_data f;
    
    pthread_mutex_t lock;
};

struct data_mission {
    bool active;

    /* TODO commands */
    //struct obj cmd_queue;
    int cmds_completed;
    int cmds_remaining;

    pthread_mutex_t lock;
};

struct data_rc {
    ctrl_val_t vel;
    ctrl_val_t rot;

    pthread_mutex_t lock;
};

/* server commands, called by server on remote command */

bool sc_get_sens(struct srv_cmd_args *a) {
    struct data_sensors *sens_data = (struct data_sensors*)a->data1;

    /* read data */
    sens_dist_t df, dr;
    sens_odom_t dist;
    pthread_mutex_lock(&sens_data->lock);
    df = sens_data->f.dist_front;
    dr = sens_data->f.dist_right;
    dist = sens_data->f.distance;
    pthread_mutex_unlock(&sens_data->lock);

    /* create string */
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';
    rsp = str_append(rsp, &buf_size, "df=%d ", df);
    rsp = str_append(rsp, &buf_size, "dr=%d ", dr);
    rsp = str_append(rsp, &buf_size, "dist=%d", dist);

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

bool sc_set_bool(struct srv_cmd_args *a) {
    int success = true;
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';

    char* bool_str = a->args[1];
    bool value;

    switch (bool_str[0]) {
        case '0':
        case 'f':
        case 'F':
            value = false;
            break;
        case '1':
        case 't':
        case 'T':
            value = true;
            break;
        default:
            success = false;
    }

    if (success) {
        bool *dst = (bool*)a->data1;
        pthread_mutex_t *lock = (pthread_mutex_t*)a->data2;
        pthread_mutex_lock(lock);
        *dst = value;
        pthread_mutex_unlock(lock);
        success = true;
        rsp = str_append(rsp, &buf_size, "setting value to %d", value);
    } else {
        rsp = str_append(rsp, &buf_size,
                         "invalid argument -- \"%s\"", bool_str);
    }

    a->resp = rsp;
    return success;
}

bool sc_set_float(struct srv_cmd_args *a) {
    int success = false;
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';

    char* float_str = a->args[1];
    char *endptr;
    float value = strtof(float_str, &endptr);

    if (endptr > float_str) {
        float *dst = (float*)a->data1;
        pthread_mutex_t *lock = (pthread_mutex_t*)a->data2;
        pthread_mutex_lock(lock);
        *dst = value;
        pthread_mutex_unlock(lock);
        success = true;
        rsp = str_append(rsp, &buf_size, "setting value to %f", value);
    } else {
        rsp = str_append(rsp, &buf_size,
                         "invalid argument -- \"%s\"", float_str);
    }

    a->resp = rsp;
    return success;
}

bool sc_bus_send_float(struct srv_cmd_args *a) {
    int success = false;
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';
    
    float value;
    char *float_str = a->args[1];
    char *endptr;
    value = strtof(float_str, &endptr);

    if (endptr > float_str) {
        success = true;
        struct bus_cmd *bc = (struct bus_cmd*)a->data1;
        bus_t *bus = (bus_t*)a->data2;
        bus_transmit_schedule(bus, bc, (unsigned char*)&value, NULL, NULL);
        rsp = str_append(rsp, &buf_size, "sending value %f", value);
    } else {
        rsp = str_append(rsp, &buf_size, "invalid arg -- \"%s\"", float_str);
    }

    a->resp = rsp;
    return success;
}

/* bus signal handlers, called by bus thread when transmission finished */

/* write received values to struct reachable from main thread */
void bsh_sens_recv(void *received, void *data) {
    struct sens_data *frame = (struct sens_data*)received;
    struct data_sensors *sens_data = (struct data_sensors*)data;
    
    pthread_mutex_lock(&sens_data->lock);
    sens_data->f = *frame;
    pthread_mutex_unlock(&sens_data->lock);
}

int main(int argc, char* args[]) {
    bool quit = false;
    pthread_mutex_t quit_lock;
    pthread_mutex_init(&quit_lock, 0);

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
    {"get_sensor",  0, &sens_data,        &sens_data.lock, *sc_get_sens},
    {"get_mission", 0, &miss_data,        &miss_data.lock, *sc_get_mission},
    {"set_mission", 1, &miss_data,        &miss_data.lock, *sc_set_mission},
    {"set_state",   1, &miss_data.active, &miss_data.lock, *sc_set_bool},
    {"shutdown",    1, &quit,             &quit_lock,      *sc_set_bool},
    {"set_vel",     1, &rc_data.vel,      &rc_data.lock,   *sc_set_float},
    {"set_rot",     1, &rc_data.rot,      &rc_data.lock,   *sc_set_float},
    {"set_vel_kp",  1, &BCC_VEL_KP,       bus,             *sc_bus_send_float},
    {"set_vel_kd",  1, &BCC_VEL_KD,       bus,             *sc_bus_send_float},
    {"set_rot_kp",  1, &BCC_ROT_KP,       bus,             *sc_bus_send_float},
    {"set_rot_kd",  1, &BCC_ROT_KD,       bus,             *sc_bus_send_float},
    };
    int cmdc = sizeof(cmds)/sizeof(*cmds);
    srv_t *srv = srv_create(inet_addr, SERVER_PORT_START, SERVER_PORT_END,
                            cmds, cmdc);
    if (!srv) return EXIT_FAILURE;

    uint8_t rsp_ctrl, rsp_sens;
    bus_receive(bus, &BCC_SYN, (void*)&rsp_ctrl);
    if (rsp_ctrl != CTRL_ACK) {
        fprintf(stderr, "WARNING: no acknowledge from ctrl\n");
    }
    bus_receive(bus, &BCC_SYN, (void*)&rsp_sens);
    if (rsp_sens != SENS_ACK) {
        fprintf(stderr, "WARNING: no acknowledge from sens\n");
    }

    char input[100];
    while (!quit) {
        /* pause loop, enable exit */
        int len = scanf("%s", input);
        if (len > 0 && input[0] == 'q') {
            pthread_mutex_lock(&quit_lock);
            quit = true;
            pthread_mutex_unlock(&quit_lock);
        }

        bus_receive_schedule(bus, &BCS_GET, bsh_sens_recv, &sens_data);

        pthread_mutex_lock(&miss_data.lock);
        if (miss_data.active) {
            pthread_mutex_unlock(&miss_data.lock);

            ctrl_val_t err_vel = 0;
            ctrl_val_t err_rot = 0;

            /* TODO img proc + mission */

            bus_transmit_schedule(bus, &BCC_VEL_ERR, (void*)&err_vel,
                    NULL, NULL);
            bus_transmit_schedule(bus, &BCC_ROT_ERR, (void*)&err_rot,
                    NULL, NULL);
        } else {
            pthread_mutex_unlock(&miss_data.lock);

            ctrl_val_t vel, rot;
            pthread_mutex_lock(&rc_data.lock);
            vel = rc_data.vel;
            rot = rc_data.rot;
            pthread_mutex_unlock(&rc_data.lock);

            bus_transmit_schedule(bus, &BCC_VEL_VAL, (void*)&vel, NULL, NULL);
            bus_transmit_schedule(bus, &BCC_ROT_VAL, (void*)&rot, NULL, NULL);
        }
    }

    srv_destroy(srv);
    bus_destroy(bus);

    return EXIT_SUCCESS;
}
