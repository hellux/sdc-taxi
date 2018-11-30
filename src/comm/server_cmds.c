#include "server_cmds.h"

#include "main.h"
#include "server.h"
#include "bus.h"
#include "objective.h"

bool sc_get_sens(struct srv_cmd_args *a) {
    pthread_mutex_t lock = ((struct data_sensors*)a->data1)->lock;
    struct sens_values *sensors = (struct sens_values*)a->data1;

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
    struct data_mission *miss_data = (struct data_mission*)a->data1;

    /* read data */
    int cmp, rem;
    pthread_mutex_lock(&miss_data->lock);
    cmp = miss_data->cmds_completed;
    pthread_mutex_unlock(&miss_data->lock);

    /* create string */
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';
    rsp = str_append(rsp, &buf_size, "completed=%d ", cmp);

    a->resp = rsp;
    return true;
}

bool sc_set_mission(struct srv_cmd_args *a) {
    int buf_size = 128;
    char *rsp = malloc(buf_size);
    rsp[0] = '\0';

    struct obj_item *queue = objq_create(a->argc-1, a->args+1);

    if (queue) {
        struct data_mission *miss_data = (struct data_mission*)a->data1;

        pthread_mutex_lock(&miss_data->lock);
        objq_destroy(miss_data->queue);
        miss_data->queue = queue;
        pthread_mutex_unlock(&miss_data->lock);

        str_append(rsp, &buf_size, "mission set successfully.");
    } else {
        str_append(rsp, &buf_size, "arguments invalid.");
    }

    a->resp = rsp;
    return queue != NULL;
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

