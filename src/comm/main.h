#ifndef main_h
#define main_h

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include <pthread.h>

#include "bus.h"
#include "server.h"
#include "server_cmds.h"
#include "ip/img_proc.h"

#include "protocol.h"

#define SERVER_PORT_START 9000
#define SERVER_PORT_END 9100

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

#endif
