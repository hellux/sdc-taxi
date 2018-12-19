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

#define SERVER_PORT_START 9000
#define SERVER_PORT_END 9100

#define WAIT_RC 1e7

static struct timespec ts_start;
#ifdef PLOT_VEL
static FILE *vel_log;
#endif

struct data_sensors {
    bus_t *bus;
    struct sens_val val;
    struct ip_res ip;
    struct ctrl_val ctrl;
    
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
    struct ctrl_val ctrl;
    struct ip_res ip;
    pthread_mutex_lock(&sens_data->lock);
    sens = sens_data->val;
    ip = sens_data->ip;
    ctrl = sens_data->ctrl;
    pthread_mutex_unlock(&sens_data->lock);

    a->resp = str_create("%.2f %.2f %.2f %.2f %.2f %.2f %.2f",
        sens.dist_front,
        sens.dist_right,
        sens.velocity,
        sens.distance,
        ip.lane_offset,
        ctrl.vel,
        ctrl.rot
    );

    return true;
}

bool sc_get_mission(struct srv_cmd_args *a) {
    a->resp = obj_remaining((obj_t*)a->data1);
    return true;
}

bool sc_set_mission(struct srv_cmd_args *a) {
    bool success = obj_set_mission((obj_t*)a->data1, a->argc-1, a->args+1, false);
    a->resp = str_create((success ? "mission set" : "invalid mission"));
    return success;
}

bool sc_append_mission(struct srv_cmd_args *a) {
    bool success = obj_set_mission((obj_t*)a->data1, a->argc-1, a->args+1, true);
    a->resp = str_create((success ? "mission appended" : "invalid mission"));
    return success;
}

bool sc_set_auto(struct srv_cmd_args *a) {
    bool state = a->args[1][0] == 'T';
    obj_set_state((obj_t*)a->data1, state);
    a->resp = str_create("setting %s mode", (state ? "auto" : "manual"));
    return true;
}

bool sc_shutdown(struct srv_cmd_args *a) {
    bool *shutdown = (bool*)a->data1;
    pthread_mutex_t *lock = (pthread_mutex_t*)a->data2;
    pthread_mutex_lock(lock);
    *shutdown = true;
    pthread_mutex_unlock(lock);
    a->resp = str_create("shutting down");
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
        bus_schedule(bus, bc, (unsigned char*)&value, NULL, NULL, false);
        a->resp = str_create("sending value %f", value);
    } else {
        a->resp = str_create("invalid arg -- \"%s\"", float_str);
    }

    return success;
}

/* bus signal handler, called by bus thread when transmission finished */
/* write received values to struct reachable from main thread */
void bsh_sens_recv(void *received, void *data) {
    static float velocity_prev = 0;
    static double update_time = 0;

    struct sens_data *sd = (struct sens_data*)received;
    struct data_sensors *sens_data = (struct data_sensors*)data;
    
    struct timespec ts_now;
    clock_gettime(CLOCK_MONOTONIC, &ts_now);
    struct timespec ts_diff = {ts_now.tv_sec - ts_start.tv_sec,
                               ts_now.tv_nsec - ts_start.tv_nsec};
    double time = ts_diff.tv_sec + ts_diff.tv_nsec/1e9;

    if (sd->dist_front > 2
     || sd->dist_front < 0
     || sd->distance < 0
     || sd->velocity > 100
     || sd->velocity < -100) {
        printf("invalid sensor values: %f %f %f %f %f\n",
                sd->dist_front,
                sd->dist_right,
                sd->distance,
                sd->velocity);
        return;
    }

    struct sens_val sens_new = {
        .dist_front = sd->dist_front,
        .dist_right = sd->dist_right,
        .distance = sd->distance,
        .velocity = sd->velocity,
        .acceleration = sd->velocity-velocity_prev,
        .time = time,
    };
    velocity_prev = sd->velocity;

    pthread_mutex_lock(&sens_data->lock);
    sens_data->val = sens_new;
    struct ctrl_val ctrl = sens_data->ctrl;
    pthread_mutex_unlock(&sens_data->lock);

    if (sd->updated || time-update_time > 0.01) {
        update_time = time;
        if (ctrl.vel < 0 || (ctrl.vel <= sd->velocity &&
            sd->velocity <= 0.3 && sens_new.acceleration <= 0)) {
            bus_schedule(sens_data->bus, &BCCS[BBC_VEL_VAL], &ctrl.vel,
                         NULL, NULL, false);
            /*
            printf("direct: wanted %f, vel %f\n", ctrl.vel, sd->velocity);
            */
        } else {
            float err = ctrl.vel - sd->velocity;
            bus_schedule(sens_data->bus, &BCCS[BBC_VEL_ERR], &err,
                         NULL, NULL, false);
            /*
            printf("regulate: wanted %f, vel %f, err %f\n", ctrl.vel, sd->velocity, err);
            */
        }
#ifdef PLOT_VEL
        fprintf(vel_log, "%f %f %f %f %f %f\n",
                sens_new.time,
                sens_new.velocity,
                ctrl.vel,
                sens_new.dist_front,
                sens_new.acceleration,
                ctrl.rot);
#endif
    }
}

int main(int argc, char* args[]) {
    clock_gettime(CLOCK_MONOTONIC, &ts_start);
#ifdef PLOT_VEL
    vel_log = fopen("vel.dat", "w");
#endif

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

    bus = bus_create();
    if (!bus) goto fail;
    sens_data.bus = bus;

    obj = obj_create();
    if (!obj) goto fail;

    struct srv_cmd cmds[] = {
    {"get_sensor",  0, &sens_data,        NULL,            *sc_get_sens},
    {"get_miss",    0, obj,               NULL,            *sc_get_mission},
    {"set_miss",    0, obj,               NULL,            *sc_set_mission},
    {"app_miss",    0, obj,               NULL,            *sc_append_mission},
    {"set_auto",   1, obj,               NULL,             *sc_set_auto},
    {"shutdown",    0, &quit,             &quit_lock,      *sc_shutdown},
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
    bus_schedule(bus, &BCSS[BBS_RST], NULL, NULL, NULL, false);
    bus_schedule(bus, &BCCS[BBC_RST], NULL, NULL, NULL, false);

    bus_schedule(bus, &BCSS[BBS_GET], NULL, bsh_sens_recv, &sens_data,
                 true);

    struct ctrl_val ctrl_prev = {0};
    while (!quit) {
        struct ctrl_val ctrl = ctrl_prev;
        pthread_mutex_lock(&sens_data.lock);
        struct sens_val sens = sens_data.val;
        pthread_mutex_unlock(&sens_data.lock);

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

            ctrl.vel = rc.vel;
            ctrl.rot = rc.rot;
        }

        /*
        if (sens.dist_front < 1) {
            float max_vel = MAX(0, sens.dist_front-0.5);
            ctrl.vel = MIN(max_vel, ctrl.vel);
        }
        */

        /* send new ctrl commands */
        pthread_mutex_lock(&sens_data.lock);
        sens_data.ctrl = ctrl;
        pthread_mutex_unlock(&sens_data.lock);
        bus_schedule(bus, &BCCS[BBC_ROT_ERR], (void*)&ctrl.rot, NULL, NULL,
                     false);
        ctrl_prev = ctrl;
    }

    bus_schedule(bus, &BCSS[BBS_RST], NULL, NULL, NULL, false);
    bus_schedule(bus, &BCCS[BBC_RST], NULL, NULL, NULL, false);

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
#ifdef PLOT_VEL
    fclose(vel_log);
#endif

    return success;
}
