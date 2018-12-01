#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <pthread.h>

#include "types.h"
#include "objective.h"
#include "bus.h"
#include "server.h"
#include "ip/img_proc.h"
#include "protocol.h"

#define SERVER_PORT_START 9000
#define SERVER_PORT_END 9100

struct data_sensors {
    struct sens_val val;
    
    pthread_mutex_t lock;
};

struct data_rc {
    float vel;
    float rot;

    pthread_mutex_t lock;
};

/* server commands */
bool sc_get_sens(struct srv_cmd_args *a) {
    pthread_mutex_t lock = ((struct data_sensors*)a->data1)->lock;
    struct sens_val *sensors = (struct sens_val*)a->data1;

    /* read data */
    float df, dr, vel, dist;
    pthread_mutex_lock(&lock);
    df = sensors->dist_front;
    dr = sensors->dist_right;
    vel = sensors->velocity;
    dist = sensors->distance;
    pthread_mutex_unlock(&lock);

    /* create string */
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';
    rsp = str_append(rsp, &buf_size, "%f ", df);
    rsp = str_append(rsp, &buf_size, "%f ", dr);
    rsp = str_append(rsp, &buf_size, "%f ", vel);
    rsp = str_append(rsp, &buf_size, "%f", dist);

    a->resp = rsp;
    return true;
}

bool sc_get_mission(struct srv_cmd_args *a) {
    int rem = obj_remaining((obj_t*)a->data1);

    /* create string */
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';
    rsp = str_append(rsp, &buf_size, "%d", rem);

    a->resp = rsp;
    return true;
}

bool sc_set_mission(struct srv_cmd_args *a) {
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';

    bool success = obj_set_mission((obj_t*)a->data1, a->argc-1, a->args+1);

    if (success) {
        str_append(rsp, &buf_size, "mission set successfully.");
    } else {
        str_append(rsp, &buf_size, "arguments invalid.");
    }

    a->resp = rsp;
    return success;
}

bool sc_set_state(struct srv_cmd_args *a) {
    bool state = a->args[1][0] == 'T';
    obj_set_state((obj_t*)a->data1, state);
    return true;
}

bool sc_set_bool(struct srv_cmd_args *a) {
    int success = true;

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

    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';

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

    char* float_str = a->args[1];
    char *endptr;
    float value = strtof(float_str, &endptr);

    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';

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
    
    float value;
    char *float_str = a->args[1];
    char *endptr;
    value = strtof(float_str, &endptr);

    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';

    if (endptr > float_str) {
        success = true;
        const struct bus_cmd *bc = (struct bus_cmd*)a->data1;
        bus_t *bus = (bus_t*)a->data2;
        bus_transmit_schedule(bus, bc, (unsigned char*)&value, NULL, NULL);
        rsp = str_append(rsp, &buf_size, "sending value %f", value);
    } else {
        rsp = str_append(rsp, &buf_size, "invalid arg -- \"%s\"", float_str);
    }

    a->resp = rsp;
    return success;
}

/* bus signal handler, called by bus thread when transmission finished */
/* write received values to struct reachable from main thread */
void bsh_sens_recv(void *received, void *data) {
    struct sens_data *frame = (struct sens_data*)received;
    struct data_sensors *sens_data = (struct data_sensors*)data;
    
    pthread_mutex_lock(&sens_data->lock);
    /*sens_data->f = *frame;*/
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
    struct data_rc rc_data = {0};
    pthread_mutex_init(&sens_data.lock, 0);
    pthread_mutex_init(&rc_data.lock, 0);

    bus_t *bus = bus_create(F_SPI);
    if (!bus) return EXIT_FAILURE;

    obj_t *obj = obj_create();
    if (!bus) return EXIT_FAILURE;

    struct srv_cmd cmds[] = {
    {"get_sensor",  0, &sens_data,        NULL,            *sc_get_sens},
    {"get_mission", 0, obj,               NULL,            *sc_get_mission},
    {"set_mission", 1, obj,               NULL,            *sc_set_mission},
    {"set_state",   1, obj,               NULL,            *sc_set_state},
    {"shutdown",    1, &quit,             &quit_lock,      *sc_set_bool},
    {"set_vel",     1, &rc_data.vel,      &rc_data.lock,   *sc_set_float},
    {"set_rot",     1, &rc_data.rot,      &rc_data.lock,   *sc_set_float},
    {"set_vel_kp",  1, &BCCS[BBC_VEL_KP], bus,             *sc_bus_send_float},
    {"set_vel_kd",  1, &BCCS[BBC_VEL_KD], bus,             *sc_bus_send_float},
    {"set_rot_kp",  1, &BCCS[BBC_ROT_KP], bus,             *sc_bus_send_float},
    {"set_rot_kd",  1, &BCCS[BBC_ROT_KD], bus,             *sc_bus_send_float},
    };
    int cmdc = sizeof(cmds)/sizeof(*cmds);
    srv_t *srv = srv_create(inet_addr, SERVER_PORT_START, SERVER_PORT_END,
                            cmds, cmdc);
    if (!srv) return EXIT_FAILURE;

    while (!quit) {
        struct ctrl_val ctrl = {0};
        pthread_mutex_lock(&sens_data.lock);
        struct sens_val sens = sens_data.val;
        pthread_mutex_unlock(&sens_data.lock);

        /*
        bus_receive_schedule(bus, &BCSS[BBS_GET], bsh_sens_recv, &sens_data);
        */

        /* determine new ctrl values */
        if (obj_active(obj)) {
            obj_execute(obj, &sens, &ctrl);
        } else {
            struct data_rc rc;
            pthread_mutex_lock(&rc_data.lock);
            rc = rc_data;
            pthread_mutex_unlock(&rc_data.lock);

            ctrl.vel.value = rc.vel;
            ctrl.rot.value = rc.rot;
            ctrl.vel.regulate = false;
            ctrl.rot.regulate = false;
        }

        /* TODO check for obstacles and override ctrl */

        /* send new ctrl commands */
        int bcc_vel = ctrl.vel.regulate ? BBC_VEL_ERR : BBC_VEL_VAL;
        int bcc_rot = ctrl.rot.regulate ? BBC_ROT_ERR : BBC_ROT_VAL;
        bus_transmit_schedule(bus, &BCCS[bcc_vel], (void*)&ctrl.vel.value,
                              NULL, NULL);
        bus_transmit_schedule(bus, &BCCS[bcc_rot], (void*)&ctrl.rot.value,
                              NULL, NULL);
    }

    obj_destroy(obj);
    srv_destroy(srv);
    bus_destroy(bus);

    return EXIT_SUCCESS;
}
