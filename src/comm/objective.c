#include "objective.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <pthread.h>

#include "ip/img_proc.h"

#define BRAKE_DIST 0.4
#define STILL_DIST 0
#define STRAIGHT 0
#define LEFT -1
#define RIGHT 1
#define STOP_VEL 0
#define SLOW_VEL 0.4
#define FULL_VEL 1

/* positions */

#define BEFORE_PREV 0
#define BEFORE_STOP 1
#define AFTER_STOP 2
#define PARKED 3

float wtd_speed(float distance, float current, float target) {
    return current+(target-current)/distance;
}

struct state {
    const struct sens_val *sens; 

    float lane_offset;

    bool stop_visible;
    float stop_dist; /* distance to visible stopline */
    double since_pass; /* seconds since last stopline pass */
    bool last_cmd;
    int pos;
};

/* objective commands */

bool cmd_ignore(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    if (s->pos == AFTER_STOP) {
        return true;
    }
    return false;
}

bool cmd_stop(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    if (c->vel.value < 0.05) {
        return true;
    } else {
        c->vel.value = wtd_speed(s->stop_dist, c->vel.value, STOP_VEL);
        return false;
    }
}

bool cmd_park(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    /* TODO only return true if last cmd */
    float cur_vel = s->sens->velocity;
    if (s->pos == AFTER_STOP) {
        c->rot.value = RIGHT;

        if (s->since_pass >= 1.5) {
            c->vel.value = STOP_VEL;
            return true;
        } else if (s->since_pass >= 1) {
            c->rot.value =STRAIGHT;
        } else if (s->since_pass >= 0.5) {
            c->rot.value = LEFT;
        }
    } else if (s->stop_dist <= BRAKE_DIST) {
        c->vel.value = wtd_speed(s->stop_dist, cur_vel, SLOW_VEL);
    } else if (s->pos == PARKED) {
        c->rot.value = LEFT;
        c->vel.value = SLOW_VEL;
        
        if (cur_vel == FULL_VEL) {
            return true;
        } else if (s->since_pass >= 1) {
            c->vel.value = FULL_VEL;
            c->rot.value = STRAIGHT;
        } else if (s->since_pass >= 0.5) {
            c->rot.value = RIGHT;
        }
        return false;
    }
    return false;
}

bool cmd_enter(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    float cur_vel = s->sens->velocity;
    if (s->pos == AFTER_STOP) {
        c->rot.value = RIGHT;
    
        if (s->since_pass >= 0.5){
            c->rot.value = STRAIGHT;
            return true;
        }
    } else if (s->stop_dist <= BRAKE_DIST) {
        c->vel.value = wtd_speed(s->stop_dist, cur_vel, SLOW_VEL);
    }
    return false;
}

bool cmd_continue(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    if (s->stop_visible) {
        i->ignore_right = true;
    } else if (s->pos == AFTER_STOP) {
        return true;
    }
    return false;
}

bool cmd_exit(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    float cur_vel = s->sens->velocity;
    if (s->pos == AFTER_STOP) {
        c->rot.value = RIGHT;
    
        if (cur_vel == FULL_VEL){
            return true;
        }
        else if (s->since_pass >= 0.5){
            c->vel.value = FULL_VEL;
            c->rot.value = STRAIGHT;
        }
    }     
    return false;
}

struct obj_cmd {
    const char name[5];
    bool (*func)(struct state *s, struct ctrl_val *c, struct ip_opt *i);
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
    double passtime;
    int pos;
};

#define CMD_IGNORE   "ignr"
#define CMD_STOP     "stop"
#define CMD_PARK     "park"
#define CMD_ENTER    "entr"
#define CMD_CONTINUE "cont"
#define CMD_EXIT     "exit"

const struct obj_cmd CMDS[] = {
    {CMD_IGNORE,   cmd_ignore},
    {CMD_STOP,     cmd_stop},
    {CMD_PARK,     cmd_park},
    {CMD_ENTER,    cmd_enter},
    {CMD_CONTINUE, cmd_continue},
    {CMD_EXIT,     cmd_exit},
};
const int CMDC = sizeof(CMDS)/sizeof(*CMDS);

/* cmd queue helpers */

void queue_destroy(struct obj_item *queue) {
    while (queue) {
        struct obj_item *prev = queue;
        queue = queue->next;
        free(prev);
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
    obj_set_mission(obj, 0, NULL);
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
        obj->passtime = 0;
        obj->pos = 0; /* TODO adjust if parked */
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
    }

    struct ip_res ip_res;
    ip_process(o->ip, &ip_res);
    if (ip_res.stopline_passed) {
        if (o->pos < AFTER_STOP) {
            o->passtime = sens->time;
            o->pos++;
        } else {
            printf("warning: invalid pass!\n");
        }
    }

    ctrl->vel.regulate = false;
    ctrl->rot.value = ip_res.lane_offset;
    ctrl->rot.regulate = true;

    if (o->current) {
        struct state state;
        state.sens = sens;
        state.lane_offset = ip_res.lane_offset;
        state.stop_visible = ip_res.stopline_visible;
        state.stop_dist = ip_res.stopline_dist;
        state.since_pass = sens->time - o->passtime;
        state.last_cmd = o->queue == NULL;
        state.pos = o->pos;

        struct ip_opt opt = {0};
        bool cmd_finished = o->current->func(&state, ctrl, &opt);
        ip_set_opt(o->ip, &opt);

        o->pos = state.pos;

        if (cmd_finished) {
            if (o->pos <= AFTER_STOP) {
                o->pos--;
            } else {
                o->pos = 0;
            }
            o->current = NULL;
        }

        finished = cmd_finished && !o->queue;
    } else {
        finished = true;
        ctrl->vel.value = 0;
    }

    /*
    if (finished) {
        pthread_mutex_lock(&o->lock);
        o->active = false;
        pthread_mutex_unlock(&o->lock);
    }
    */
}
