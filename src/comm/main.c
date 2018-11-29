#include "main.h"

#include <string.h>

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
    {"set_mission", 1, &miss_data,        NULL,            *sc_set_mission},
    {"set_state",   1, &miss_data.active, &miss_data.lock, *sc_set_bool},
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

    ip_t *ip = ip_init();

    struct data_ctrl ctrl_prev = {0};

    struct sens_values sens;
    struct data_ctrl ctrl;
    struct ip_res ip_res;

    struct car_state state;
    state.sens = &sens;
    state.ip = &ip_res;

    struct obj_item *obj_current = NULL;
    void *obj_data = NULL;

    //char input[100];
    while (!quit) {
        ctrl = ctrl_prev;
        pthread_mutex_lock(&sens_data.lock);
        sens = sens_data.val;
        pthread_mutex_unlock(&sens_data.lock);

        /*
        bus_receive_schedule(bus, &BCSS[BBS_GET], bsh_sens_recv, &sens_data);
        */

        pthread_mutex_lock(&miss_data.lock);
        bool mission = miss_data.active;
        pthread_mutex_unlock(&miss_data.lock);

        /* determine new ctrl values */
        if (mission) {
            pthread_mutex_lock(&miss_data.lock);
            if (!obj_current) {
                if (miss_data.queue) {
                    obj_current = miss_data.queue;
                    miss_data.queue = obj_current->next;
                } else {
                    miss_data.active = false;
                    mission = false;
                }
            }
            pthread_mutex_unlock(&miss_data.lock);

            if (mission) {
                ip_process(ip, &ip_res);

                ctrl.vel.value = NORMAL_SPEED;
                ctrl.vel.regulate = false;
                ctrl.rot.value = ip_res.error;
                ctrl.rot.regulate = true;

                if (ip_res.stopline_found) {
                    if (obj_current->obj->func(&state, &ctrl, obj_data)) {
                        obj_current = NULL;
                        obj_data = NULL;
                    }
                }
            } else {
                ctrl.vel.value = 0;
                ctrl.rot.value = 0;
                ctrl.vel.regulate = false;
                ctrl.rot.regulate = false;
            }
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

        /* send new ctrl commands if regulating or values are changed */
        if (ctrl.vel.regulate || ctrl.rot.regulate ||
                memcmp(&ctrl, &ctrl_prev, sizeof(ctrl)) != 0) {
            ctrl_prev = ctrl;
            int bcc_vel = ctrl.vel.regulate ? BBC_VEL_ERR : BBC_ROT_VAL;
            int bcc_rot = ctrl.rot.regulate ? BBC_ROT_ERR : BBC_ROT_VAL;
            bus_transmit_schedule(bus, &BCCS[bcc_vel], (void*)&ctrl.vel.value,
                                  NULL, NULL);
            bus_transmit_schedule(bus, &BCCS[bcc_rot], (void*)&ctrl.rot.value,
                                  NULL, NULL);
        }
    }

    srv_destroy(srv);
    bus_destroy(bus);

    return EXIT_SUCCESS;
}
