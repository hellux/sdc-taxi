#include "objective.h"

#include <string.h>
#include <stdlib.h>

#include <pthread.h>

#include "ip/img_proc.h"

#define BRAKE_DIST 0.4 //distance from line when braking starts          (cm)
#define STILL_DIST 0  //distance from line when car should be still     (cm)
#define STRAIGHT 0
#define LEFT -1
#define RIGHT 1
#define STOP_VEL 0
#define SLOW_VEL 0.4
#define FULL_VEL 1

#define SEC 1e3 /* milliseconds per second */

float wtd_speed(float distance, float current, float target) {
    return (target-current)/distance;
}

struct state {
    const struct sens_val *sens; 

    float lane_offset;
    bool lane_found;

    float stopline_dist; /* meters */
    unsigned stopline_since; /* milliseconds */
    bool stopline_passed;
};

/* objective commands */

bool cmd_ignore(const struct state *s, struct ctrl_val *c) {
    if (s->stopline_passed) {
        return true;
    }
    return false;
}

bool cmd_stop(const struct state *s, struct ctrl_val *c) {
    float cur_vel = s->sens->velocity;
    if (cur_vel == STOP_VEL)
        return true;
    else if (s->stopline_dist <= BRAKE_DIST) {
        c->vel.value = wtd_speed(s->stopline_dist, cur_vel, STOP_VEL);
    }
    return false;
}

bool cmd_park(const struct state *s, struct ctrl_val *c) {
    float cur_vel = s->sens->velocity;
    if (s->stopline_passed) {
        c->rot.value = RIGHT;

        if (s->stopline_since >= SEC*1.5){
            c->vel.value = STOP_VEL;
            return true;
        }
        else if (s->stopline_since >= SEC)
            c->rot.value =STRAIGHT;
        else if (s->stopline_since >= SEC*0.5)
            c->rot.value = LEFT;
    } else if (s->stopline_dist <= BRAKE_DIST) {
        c->vel.value = wtd_speed(s->stopline_dist, cur_vel, SLOW_VEL);
    }
    return false;
}

bool cmd_unpark(const struct state *s, struct ctrl_val *c) {
    float cur_vel = s->sens->velocity; 
    c->rot.value = LEFT;
    c->vel.value = SLOW_VEL;
    
    if (cur_vel == FULL_VEL)
        return true;
    else if (s->stopline_since >= SEC){
        c->vel.value = FULL_VEL;
        c->rot.value = STRAIGHT;
    }
    else if (s->stopline_since >= SEC*0.5)
        c->rot.value = RIGHT;
    return false;
}

bool cmd_enter(const struct state *s, struct ctrl_val *c) {
    float cur_vel = s->sens->velocity;
    if (s->stopline_passed) {
        c->rot.value = RIGHT;
    
        if (s->stopline_since >= SEC*0.5){
            c->rot.value = STRAIGHT;
            return true;
        }
    } else if (s->stopline_dist <= BRAKE_DIST) {
        c->vel.value = wtd_speed(s->stopline_dist, cur_vel, SLOW_VEL);
    }
    return false;
}

bool cmd_exit(const struct state *s, struct ctrl_val *c) {
    float cur_vel = s->sens->velocity;
    if (s->stopline_passed) {
        c->rot.value = RIGHT;
    
        if (cur_vel == FULL_VEL){
            return true;
        }
        else if (s->stopline_since >= SEC*0.5){
            c->vel.value = FULL_VEL;
            c->rot.value = STRAIGHT;
        }
    }     
    return false;
}

struct obj_cmd {
    const char name[5];
    bool (*func)(const struct state *s, struct ctrl_val *c);
};

struct obj_item {
    struct obj_cmd const *cmd;
    struct obj_item *next;
};

struct obj {
    ip_t *ip;

    /* current mission state */
    bool active;
    struct obj_cmd const *current;
    struct obj_item *queue;
    pthread_mutex_t lock;

    /* current command state */
    long unsigned stopline_passtime;
    bool stopline_found;
    bool stopline_passed;
};

#define CMD_IGNORE "ignr"
#define CMD_STOP   "stop"
#define CMD_PARK   "park"
#define CMD_UNPARK "uprk"
#define CMD_ENTER  "entr"
#define CMD_EXIT   "exit"

const struct obj_cmd CMDS[] = {
    {CMD_IGNORE, cmd_ignore},
    {CMD_STOP,   cmd_stop},
    {CMD_PARK,   cmd_park},
    {CMD_UNPARK, cmd_unpark},
    {CMD_ENTER,  cmd_enter},
    {CMD_EXIT,   cmd_exit},
};
const int CMDC = sizeof(CMDS)/sizeof(*CMDS);

/* cmd queue helpers */

void queue_destroy(struct obj_item *queue) {
    while (queue) {
        struct obj_item *prev = queue;
        queue = queue->next;
        free(queue);
    }
}

struct obj_item *queue_create(int cmdc, char **cmds) {
    bool valid = true;
    struct obj_item *root = NULL;
    struct obj_item **prev_ptr = &root;
    struct obj_item *current;

    for (int i = 0; i < cmdc; i++) {
        current = calloc(1, sizeof(*current));
        *prev_ptr = current;

        for (int j = 0; j < CMDC; j++) {
            if (strcmp(cmds[i], CMDS[j].name) == 0) {
                current->cmd = &CMDS[j];
                break;
            }
        }
        if (!current->cmd) {
            valid = false;
            break;
        }
    }

    if (valid) {
        return root;
    } else {
        queue_destroy(root);
        return NULL;
    }
}

/* external functions */

struct obj *obj_create(void) {
    struct obj *obj = calloc(1, sizeof(*obj));
    pthread_mutex_init(&obj->lock, 0);
    obj->ip = ip_init();

    return obj;
}

void obj_destroy(struct obj *obj) {
    /* TODO properly destroy, avoid multithreading issues */
    pthread_mutex_destroy(&obj->lock);
    ip_destroy(obj->ip);
    queue_destroy(obj->queue);

    free(obj);
}

bool obj_active(obj_t *obj) {
    pthread_mutex_lock(&obj->lock);
    bool active = obj->active;
    pthread_mutex_unlock(&obj->lock);

    return active;
}

void obj_set_state(obj_t *obj, bool state) {
    pthread_mutex_lock(&obj->lock);
    obj->active = state;
    pthread_mutex_unlock(&obj->lock);
}

int obj_remaining(obj_t *obj) {
    int remaining = 0;
    struct obj_item *current;

    pthread_mutex_lock(&obj->lock);
    current = obj->queue;
    while (current) {
        current = current->next;
        remaining++;
    }
    pthread_mutex_unlock(&obj->lock);

    return remaining;
}

bool obj_set_mission(obj_t *obj, int cmdc, char **cmds) {
    struct obj_item *queue = queue_create(cmdc, cmds);

    if (queue) {
        pthread_mutex_lock(&obj->lock);
        queue_destroy(obj->queue);
        obj->queue = queue;
        pthread_mutex_unlock(&obj->lock);
        return true;
    } else {
        return false;
    }
}

/* execute objective command */
void obj_execute(struct obj *o, const struct sens_val *sens,
                 struct ctrl_val *ctrl) {
    bool finished = false;

    /* get next command if needed */
    if (!o->current && o->queue) {
        o->current = o->queue->cmd;
        o->queue = o->queue->next;

        o->stopline_passed = false;
    }

    if (o->current) {
        struct ip_res ip_res;
        ip_process(o->ip, &ip_res);
        if (ip_res.stopline_found)
            o->stopline_found = true;

        ctrl->vel.value = FULL_VEL;
        ctrl->vel.regulate = false;
        ctrl->rot.value = ip_res.lane_offset;
        ctrl->rot.regulate = true;

        if (o->stopline_found) {
            if (!o->stopline_passed && ip_res.stopline_dist <= 0) {
                o->stopline_passed = true;
                o->stopline_passtime = sens->time;
            }

            struct state state;
            state.sens = sens;
            state.lane_offset = ip_res.lane_offset;
            state.lane_found = ip_res.lane_found;
            state.stopline_dist = ip_res.stopline_dist;
            state.stopline_since = sens->time - o->stopline_passtime;
            state.stopline_passed = o->stopline_passed;

            bool cmd_finished = o->current->func(&state, ctrl);
            if (cmd_finished) {
                o->stopline_found = false;
                o->stopline_passed = false;
                o->current = NULL;
            }
            finished = cmd_finished && !o->queue;
        }
    } else {
        finished = true;
    }

    if (finished) {
        pthread_mutex_lock(&o->lock);
        o->active = false;
        pthread_mutex_unlock(&o->lock);
    }
}

