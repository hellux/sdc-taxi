#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include <pthread.h>

#include "bus.h"
#include "server.h"
#include "ip/img_proc.h"

#define SERVER_PORT_START 9000
#define SERVER_PORT_END 9100

#define SLAVE_SENS 10
#define SLAVE_CTRL 11

#define F_SPI 4000000

typedef uint8_t sens_dist_t;
typedef uint8_t sens_rot_t;
typedef float   reg_const_t;
typedef float   err_val_t;

/* format of data sent from sensor via bus */
struct sens_data_frame {
    sens_dist_t dist_front;
    sens_dist_t dist_right;
    sens_rot_t rotations;
};

/* flags for bus command;
 *  examples:
 *      write speed =   0000 0011 = 3
 *      write turn kp = 0000 0101 = 5 */
#define BF_WRITE 1
#define BF_SPEED 2
#define BF_REG   4
#define BF_KD    8

/* bus cmd bytes */
#define BCB_SPEED    BF_WRITE|BF_SPEED
#define BCB_SPEED_KD BF_WRITE|BF_SPEED|BF_REG|BF_KD
#define BCB_SPEED_KP BF_WRITE|BF_SPEED|BF_REG
#define BCB_TURN     BF_WRITE
#define BCB_TURN_KD  BF_WRITE|         BF_REG|BF_KD
#define BCB_TURN_KP  BF_WRITE|         BF_REG
#define BCB_GET_SENS 16
#define BCB_RESET    32

/* bus commands for ctrl */
const struct bus_cmd BC_SPEED = {
    BCB_SPEED, SLAVE_CTRL, sizeof(err_val_t)
};
const struct bus_cmd BC_SPEED_KD = {
    BCB_SPEED_KD, SLAVE_CTRL, sizeof(reg_const_t)
};
const struct bus_cmd BC_SPEED_KP = {
    BCB_SPEED_KP, SLAVE_CTRL, sizeof(reg_const_t)
};
const struct bus_cmd BC_TURN = {
    BCB_TURN, SLAVE_CTRL, sizeof(err_val_t)
};
const struct bus_cmd BC_TURN_KD = {
    BCB_TURN_KD, SLAVE_CTRL, sizeof(reg_const_t)
};
const struct bus_cmd BC_TURN_KP = {
    BCB_TURN_KP, SLAVE_CTRL, sizeof(reg_const_t)
};
const struct bus_cmd BC_RST_CTRL = {
    BCB_RESET, SLAVE_CTRL, 0
};

/* bus commands for sens */
const struct bus_cmd BC_GET_SENS = {
    BCB_GET_SENS, SLAVE_SENS, sizeof(struct sens_data_frame)
};
const struct bus_cmd BC_RST_SENS = {
    BCB_RESET, SLAVE_CTRL, 0
};

/* data from sensors stored on pi */
struct data_sensors {
    sens_dist_t dist_front;
    sens_dist_t dist_right;
    unsigned rotations;
    
    pthread_mutex_t lock;
};

struct data_mission {
    int cmds_completed;
    int cmds_remaining;

    pthread_mutex_t lock;
};

/* server commands, called by server on remote command */

bool sc_get_sens(struct srv_cmd_args *a) {
    return true;
}

bool sc_get_mission(struct srv_cmd_args *a) {
    return true;
}

bool sc_set_mission(struct srv_cmd_args *a) {
    return true;
}

bool sc_set_bool(struct srv_cmd_args *a) {
    return true;
}

bool sc_set_float(struct srv_cmd_args *a) {
    return true;
}

bool sc_bus_send_float(struct srv_cmd_args *a) {
    return true;
}

/* bus signal handlers, called by bus thread when transmission finished */

/* write received values to struct reachable from main thread */
void bsh_sens_recv(unsigned char* received, void *data) {
    struct sens_data_frame *frame = (struct sens_data_frame*)received;
    struct data_sensors *dst = (struct data_sensors*)data;
    
    pthread_mutex_lock(&dst->lock);
    dst->dist_front = frame->dist_front;
    dst->dist_right = frame->dist_right;
    dst->rotations += frame->rotations;
    pthread_mutex_unlock(&dst->lock);
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
    pthread_mutex_init(&sens_data.lock, 0);
    pthread_mutex_init(&miss_data.lock, 0);

    struct reg_consts;
    struct srv_cmd cmds[] = {
        {"get_sensor_data", 0, &sens_data,  NULL, *sc_get_sens},
        {"get_mission",     0, &miss_data,  NULL, *sc_get_mission},
        {"set_mission",     0, NULL,        NULL, *sc_set_mission},
        {"set_state",       0, NULL,        NULL, *sc_set_bool},
        {"set_speed_delta", 0, NULL,        NULL, *sc_set_float},
        {"set_speed_kp",    0, NULL,        NULL, *sc_set_float},
        {"set_speed_kd",    0, NULL,        NULL, *sc_set_float},
        {"set_turn_delta",  0, NULL,        NULL, *sc_set_float},
        {"set_turn_kp",     0, NULL,        NULL, *sc_set_float},
        {"set_turn_kd",     0, NULL,        NULL, *sc_set_float},
    };
    int cmdc = sizeof(cmds)/sizeof(*cmds);
    srv_t *srv = srv_create(inet_addr, SERVER_PORT_START, SERVER_PORT_END,
                            cmds, cmdc);
    if (!srv) return EXIT_FAILURE;

    bus_t *bus = bus_create(F_SPI);
    if (!bus) return EXIT_FAILURE;

    struct sens_data_frame frame;
    bus_receive(bus, &BC_GET_SENS, (unsigned char*)&frame);
    printf("dist: %d, rot: %d\n", frame.dist_front, frame.rotations);

    /*
    bus_transmit(bus, 0, 7, msg, 8);
    bus_transmit_schedule(bus, 0, 7, msg, 8, NULL, NULL);
    */

    char input[100];
    while (!quit) {
        bus_receive_schedule(bus, &BC_GET_SENS, bsh_sens_recv, &sens_data);

        /* pause loop, enable exit */
        scanf("%s", input);
        if (input[0] == 'q')
            quit = true;

        pthread_mutex_lock(&sens_data.lock);
        printf("dist_front: %d, dist_right: %d, rotations: %d\n",
                sens_data.dist_front, sens_data.dist_right,
                sens_data.rotations);
        pthread_mutex_unlock(&sens_data.lock);

        /* TODO double err = ip_process() */
    }

    bus_destroy(bus);
    srv_destroy(srv);

    return EXIT_SUCCESS;
}
