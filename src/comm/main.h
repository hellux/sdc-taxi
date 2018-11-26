#ifndef main_h
#define main_h

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include <pthread.h>

#define SERVER_PORT_START 9000
#define SERVER_PORT_END 9100

struct ctrl_pair {
    float vel;
    float rot;
};

struct sens_values {
    float dist_front;
    float dist_right;
    float distance;
    float velocity;
};

/* data from sensors stored on pi */
struct data_sensors {
    struct sens_values val;
    
    pthread_mutex_t lock;
};

struct data_mission {
    bool active;

    struct obj_item *queue;
    int cmds_completed;

    pthread_mutex_t lock;
};

struct data_rc {
    struct ctrl_pair val;

    pthread_mutex_t lock;
};

#endif
