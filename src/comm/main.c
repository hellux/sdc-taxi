#include "main.h"

#include "objective.h"
#include "bus.h"
#include "server.h"
#include "server_cmds.h"
#include "ip/img_proc.h"
#include "protocol.h"

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
    struct data_mission miss_data = {0};
    struct data_rc rc_data = {0};
    pthread_mutex_init(&sens_data.lock, 0);
    pthread_mutex_init(&miss_data.lock, 0);
    pthread_mutex_init(&rc_data.lock, 0);

    bus_t *bus = bus_create(F_SPI);
    if (!bus) return EXIT_FAILURE;

    struct srv_cmd cmds[] = {
    {"get_sensor",  0, &sens_data,        NULL,            *sc_get_sens},
    {"get_mission", 0, &miss_data,        NULL,            *sc_get_mission},
    {"set_mission", 1, &miss_data,        &miss_data.lock, *sc_set_mission},
    {"set_state",   1, &miss_data.active, NULL,            *sc_set_bool},
    {"shutdown",    1, &quit,             &quit_lock,      *sc_set_bool},
    {"set_vel",     1, &rc_data.val.vel,  &rc_data.lock,   *sc_set_float},
    {"set_rot",     1, &rc_data.val.rot,  &rc_data.lock,   *sc_set_float},
    {"set_vel_kp",  1, &BCCS[BBC_VEL_KP], bus,             *sc_bus_send_float},
    {"set_vel_kd",  1, &BCCS[BBC_VEL_KD], bus,             *sc_bus_send_float},
    {"set_rot_kp",  1, &BCCS[BBC_ROT_KP], bus,             *sc_bus_send_float},
    {"set_rot_kd",  1, &BCCS[BBC_ROT_KD], bus,             *sc_bus_send_float},
    };
    int cmdc = sizeof(cmds)/sizeof(*cmds);
    srv_t *srv = srv_create(inet_addr, SERVER_PORT_START, SERVER_PORT_END,
                            cmds, cmdc);
    if (!srv) return EXIT_FAILURE;

    struct data_rc rc_prev = {0};

    char input[100];
    while (!quit) {
        /* pause loop, enable exit */
        printf("> ");
        int len = scanf("%s", input);
        if (len > 0 && input[0] == 'q') {
            pthread_mutex_lock(&quit_lock);
            quit = true;
            pthread_mutex_unlock(&quit_lock);
        }

        /*
        bus_receive_schedule(bus, &BCSS[BBS_GET], bsh_sens_recv, &sens_data);
        */

        pthread_mutex_lock(&miss_data.lock);
        if (miss_data.active) {
            pthread_mutex_unlock(&miss_data.lock);

            ctrl_val_t err_vel = 0;
            ctrl_val_t err_rot = 0;

            /* TODO img proc + mission */

            bus_transmit_schedule(bus, &BCCS[BBC_VEL_ERR], (void*)&err_vel,
                    NULL, NULL);
            bus_transmit_schedule(bus, &BCCS[BBC_ROT_ERR], (void*)&err_rot,
                    NULL, NULL);
        } else {
            pthread_mutex_unlock(&miss_data.lock);

            struct data_rc rc_local;
            pthread_mutex_lock(&rc_data.lock);
            rc_local = rc_data;
            pthread_mutex_unlock(&rc_data.lock);

            if (rc_local.val.vel != rc_prev.val.vel) {
                bus_transmit_schedule(bus, &BCCS[BBC_VEL_VAL],
                                      (void*)&rc_local.val.vel, NULL, NULL);
            }
            if (rc_local.val.rot != rc_prev.val.rot) {
                bus_transmit_schedule(bus, &BCCS[BBC_ROT_VAL],
                                      (void*)&rc_local.val.rot, NULL, NULL);
            }
            rc_prev = rc_local;
        }
    }

    srv_destroy(srv);
    bus_destroy(bus);

    return EXIT_SUCCESS;
}
