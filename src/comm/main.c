#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <pthread.h>
#include <time.h>

#include "types.h"
#include "objective.h"
#include "bus.h"
#include "server.h"
#include "ip/img_proc.h"
#include "protocol.h"

#define F_SPI 1e6

#define SERVER_PORT_START 9000
#define SERVER_PORT_END 9100

#define WAIT_RC 1e7

static struct timespec ts_start;

struct data_sensors {
    struct sens_val val;
    struct ip_res ip;
    
    pthread_mutex_t lock;
};

struct data_rc {
    float vel;
    float rot;

    pthread_mutex_t lock;
};

/* server commands */
bool sc_get_sens(struct srv_cmd_args *a) {
    struct data_sensors *sens_data = (struct data_sensors*)a->data1;

    struct sens_val sens;
    struct ip_res ip;
    pthread_mutex_lock(&sens_data->lock);
    sens = sens_data->val;
    ip = sens_data->ip;
    pthread_mutex_unlock(&sens_data->lock);

    a->resp = str_create("%f %f %f %f %f",
        sens.dist_front, sens.dist_right, sens.velocity,
        sens.distance, ip.lane_offset);

    return true;
}

bool sc_get_mission(struct srv_cmd_args *a) {
    int rem = obj_remaining((obj_t*)a->data1);
    a->resp = str_create("%d", rem);
    return true;
}

bool sc_set_mission(struct srv_cmd_args *a) {
    bool success = obj_set_mission((obj_t*)a->data1, a->argc-1, a->args+1);
    a->resp = str_create((success ? "mission set" : "invalid mission"));
    return success;
}

bool sc_set_state(struct srv_cmd_args *a) {
    bool state = a->args[1][0] == 'T';
    obj_set_state((obj_t*)a->data1, state);
    return true;
}

bool sc_set_bool(struct srv_cmd_args *a) {
    bool *dst = (bool*)a->data1;
    pthread_mutex_t *lock = (pthread_mutex_t*)a->data2;

    bool value = a->args[1][0] == 'T';
    pthread_mutex_lock(lock);
    *dst = value;
    pthread_mutex_unlock(lock);

    a->resp = str_create("setting value to %d", value);
    return true;
}

bool sc_set_float(struct srv_cmd_args *a) {
    int success = false;

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
        a->resp = str_create("setting value to %f", value);
    } else {
        a->resp = str_create("invalid argument -- \"%s\"", float_str);
    }

    return success;
}

bool sc_bus_send_float(struct srv_cmd_args *a) {
    int success = false;
    
    float value;
    char *float_str = a->args[1];
    char *endptr;
    value = strtof(float_str, &endptr);

    if (endptr > float_str) {
        success = true;
        const struct bus_cmd *bc = (struct bus_cmd*)a->data1;
        bus_t *bus = (bus_t*)a->data2;
        bus_schedule(bus, bc, (unsigned char*)&value, NULL, NULL);
        a->resp = str_create("sending value %f", value);
    } else {
        a->resp = str_create("invalid arg -- \"%s\"", float_str);
    }

    return success;
}

/* bus signal handler, called by bus thread when transmission finished */
/* write received values to struct reachable from main thread */
void bsh_sens_recv(void *received, void *data) {
    struct sens_data *sd = (struct sens_data*)received;
    struct data_sensors *sens_data = (struct data_sensors*)data;
    
    struct timespec ts_now;
    clock_gettime(CLOCK_MONOTONIC, &ts_now);
    struct timespec ts_diff = {ts_now.tv_sec - ts_start.tv_sec,
                               ts_now.tv_nsec - ts_start.tv_nsec};
    double time = ts_diff.tv_sec + ts_diff.tv_nsec/1e9;

    struct sens_val sens_new = {
        .dist_front = sd->dist_front,
        .dist_right = sd->dist_right,
        .distance = sd->distance,
        .velocity = sd->velocity,
        .time = time,
    };

    pthread_mutex_lock(&sens_data->lock);
    sens_data->val = sens_new;
    pthread_mutex_unlock(&sens_data->lock);
}

int main(int argc, char* args[]) {
    clock_gettime(CLOCK_MONOTONIC, &ts_start);

    int success = EXIT_SUCCESS;
    bool quit = false;
    pthread_mutex_t quit_lock;
    pthread_mutex_init(&quit_lock, 0);

    bus_t *bus = NULL;
    obj_t *obj = NULL;
    srv_t *srv = NULL;

    struct data_sensors sens_data = {0};
    struct data_rc rc_data = {0};
    pthread_mutex_init(&sens_data.lock, 0);
    pthread_mutex_init(&rc_data.lock, 0);

    const char *inet_addr = args[1];
    if (!inet_addr) {
        fprintf(stderr, "error: no IP address specified\n");
        goto fail;
    }

    bus = bus_create(F_SPI);
    if (!bus) goto fail;

    obj = obj_create();
    if (!obj) goto fail;

    struct srv_cmd cmds[] = {
    {"get_sensor",  0, &sens_data,        NULL,            *sc_get_sens},
    {"get_mission", 0, obj,               NULL,            *sc_get_mission},
    {"set_mission", 0, obj,               NULL,            *sc_set_mission},
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
    srv = srv_create(inet_addr, SERVER_PORT_START, SERVER_PORT_END,
                     cmds, cmdc);
    if (!srv) goto fail;

    bus_sync(bus);
    bus_schedule(bus, &BCSS[BBS_RST], NULL, NULL, NULL);
    bus_schedule(bus, &BCCS[BBC_RST], NULL, NULL, NULL);

    while (!quit) {
        struct ctrl_val ctrl = {0};
        pthread_mutex_lock(&sens_data.lock);
        struct sens_val sens = sens_data.val;
        pthread_mutex_unlock(&sens_data.lock);

        bus_schedule(bus, &BCSS[BBS_GET], NULL, bsh_sens_recv, &sens_data);

        /* determine new ctrl values */
        if (obj_active(obj)) {
            struct ip_res ip_res;
            obj_execute(obj, &sens, &ctrl, &ip_res);
            pthread_mutex_lock(&sens_data.lock);
            sens_data.ip = ip_res;
            pthread_mutex_unlock(&sens_data.lock);
        } else {
            struct timespec ts_wait = {0, WAIT_RC};
            nanosleep(&ts_wait, NULL);
            struct data_rc rc;
            pthread_mutex_lock(&rc_data.lock);
            rc = rc_data;
            pthread_mutex_unlock(&rc_data.lock);

            ctrl.vel.value = rc.vel;
            ctrl.rot.value = rc.rot;
            ctrl.vel.regulate = false;
            ctrl.rot.regulate = false;
        }

        /*
        if (sens.distance < 1) {
            ctrl.vel.value = (1+.3-sens.distance)*sens.velocity;
            ctrl.vel.regulate = true;
        }
        */

        /* send new ctrl commands */
        int bcc_vel = BBC_VEL_VAL;
        int bcc_rot = ctrl.rot.regulate ? BBC_ROT_ERR : BBC_ROT_VAL;

        bus_schedule(bus, &BCCS[bcc_vel], (void*)&ctrl.vel.value, NULL, NULL);
        bus_schedule(bus, &BCCS[bcc_rot], (void*)&ctrl.rot.value, NULL, NULL);
    }

    /*
    bus_schedule(bus, &BCSS[BBS_RST], NULL, NULL, NULL);
    bus_schedule(bus, &BCCS[BBC_RST], NULL, NULL, NULL);
    */

    goto exit;
fail:
    success = EXIT_FAILURE;
exit:
    pthread_mutex_destroy(&quit_lock);
    pthread_mutex_destroy(&sens_data.lock);
    pthread_mutex_destroy(&rc_data.lock);
    srv_destroy(srv);
    obj_destroy(obj);
    bus_destroy(bus);

    return success;
}
