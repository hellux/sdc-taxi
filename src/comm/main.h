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

#define NORMAL_SPEED 0.5

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
    float vel;
    float rot;

    pthread_mutex_t lock;
};

struct val {
    float value;
    bool regulate;
};

struct data_ctrl {
    struct val vel;
    struct val rot;
};

struct car_state {
    const struct sens_values *sens; 
    const struct ip_res *ip;

    bool in_roundabout;
};

#endif
