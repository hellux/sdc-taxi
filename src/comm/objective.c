#include "objective.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <pthread.h>

#include "ip/img_proc.h"

#define BRAKE_DIST 20
#define STILL_DIST 0
#define STRAIGHT 0
#define LEFT -1
#define RIGHT 1
#define STOP_VEL 0
#define SLOW_VEL 0.4
#define FULL_VEL 0.5

/* initial positions */

#define BEFORE_PREV 0
#define BEFORE_STOP 1
#define AFTER_STOP 2

float wtd_speed(float distance, float current, float target) {
    return current+(target-current)/distance;
}

struct state {
    const struct sens_val *sens; 

    float lane_offset;

    bool stop_visible; /* stopline is visible */
    float stop_dist; /* distance to visible stopline */
    bool last_cmd; /* command is last in queue */

    int pos;
    double postime; /* seconds since pos change */
    double posdist; /* meters since pos change */
};

/* objective commands */

bool cmd_ignore(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    if (s->pos >= AFTER_STOP) {
        return true;
    }

    return false;
}

bool cmd_stop(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    if (c->vel.value < 0.05) {
        return true;
    } else if (s->stop_visible) { // && s->stop_dist <= BRAKE_DIST) {
        c->vel.value = 0;
        c->vel.regulate = true;
    }
    return false;
}

#define PARKING 3
#define PARKED 4
#define UNPARKING 5

bool cmd_park(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    /* TODO only return true if last cmd */
    float cur_vel = s->sens->velocity;
    if (s->pos == AFTER_STOP) {
        c->rot.value = RIGHT;

        if (s->postime >= 1.5) {
            c->vel.value = STOP_VEL;
            return true;
        } else if (s->postime >= 1) {
            c->rot.value =STRAIGHT;
        } else if (s->postime >= 0.5) {
            c->rot.value = LEFT;
        }
    } else if (s->stop_dist <= BRAKE_DIST) {
        c->vel.value = wtd_speed(s->stop_dist, cur_vel, SLOW_VEL);
    } else if (s->pos == PARKED) {
        c->rot.value = LEFT;
        c->vel.value = SLOW_VEL;
        
        if (cur_vel == FULL_VEL) {
            return true;
        } else if (s->postime >= 1) {
            c->vel.value = FULL_VEL;
            c->rot.value = STRAIGHT;
        } else if (s->postime >= 0.5) {
            c->rot.value = RIGHT;
        }
        return false;
    }
    return false;
}

bool cmd_enter(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    if (s->pos <= BEFORE_STOP && s->stop_visible) {
        i->ignore_left = true;
    } else if (s->pos >= AFTER_STOP) {
        return true;
    }
    return false;
}

bool cmd_continue(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    if (s->pos <= BEFORE_STOP && s->stop_visible) {
        i->ignore_right = true;
    } else if (s->pos >= AFTER_STOP) {
        return true;
    }
    return false;
}

bool cmd_exit(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    /* begin holding right before stopline */
    if ((s->pos <= BEFORE_STOP && s->stop_visible) || s->pos >= AFTER_STOP)
        i->ignore_left = true;

    /* finish after exit */
    if (s->pos >= AFTER_STOP && s->postime > 1)
        return true;

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
    double passtime; /* time when passing stopline or changing pos */
    double passdist;
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
        prev_ptr = &current->next;

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
        struct obj_item *curr = root;
        while (curr) {
            curr = curr->next;
        }
        return root;
    } else {
        queue_destroy(root);
        return NULL;
    }
}

/* external functions */

struct obj *obj_create(void) {
    ip_t *ip = NULL;
#ifdef IP
    ip = ip_init();
    if (!ip)
        return NULL;
#endif
    struct obj *obj = calloc(1, sizeof(*obj));
    pthread_mutex_init(&obj->lock, 0);
    obj_set_mission(obj, 0, NULL);
    obj->ip = ip;

    return obj;
}

void obj_destroy(struct obj *obj) {
    /* TODO properly destroy, avoid multithreading issues */
    if (obj) {
        pthread_mutex_destroy(&obj->lock);
#ifdef IP
        ip_destroy(obj->ip);
#endif
        queue_destroy(obj->queue);

        free(obj);
    }
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

    if (cmdc == 0 || queue) {
        pthread_mutex_lock(&obj->lock);
        queue_destroy(obj->queue);
        obj->queue = queue;
        obj->passtime = 0;
        obj->pos = BEFORE_STOP;
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
#ifdef IP
    ip_process(o->ip, &ip_res);
#endif
    if (ip_res.stopline_passed) {
        o->pos++;
        o->passtime = sens->time;
        o->passdist = sens->distance;
        printf("pos: %d, obj: %s\n", o->pos, o->current->name);
    }

    ctrl->vel.value = 0;
    ctrl->vel.regulate = false;
    ctrl->rot.value = ip_res.lane_offset;
    ctrl->rot.regulate = true;

    if (o->current) {
        ctrl->vel.value = FULL_VEL;

        struct state state;
        state.sens = sens;
        state.lane_offset = ip_res.lane_offset;
        state.stop_visible = ip_res.stopline_visible;
        state.stop_dist = ip_res.stopline_dist;
        state.postime = sens->time - o->passtime;
        state.posdist = sens->distance - o->passdist;
        state.last_cmd = o->queue == NULL;
        state.pos = o->pos;

        struct ip_opt opt = {0};
        bool cmd_finished = o->current->func(&state, ctrl, &opt);
#ifdef IP
        ip_set_opt(o->ip, &opt);
#endif

        if (o->pos != state.pos) {
            o->pos = state.pos;
            o->passtime = sens->time;
            o->passdist = sens->distance;
        }

        if (cmd_finished) {
            printf("finished cmd\n");
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

    if (finished) {
        pthread_mutex_lock(&o->lock);
        o->active = false;
        pthread_mutex_unlock(&o->lock);
    }
}
