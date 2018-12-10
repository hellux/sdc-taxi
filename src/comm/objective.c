#include "objective.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <pthread.h>

#define PICKUP_TIME 5 /* seconds to wait at pickup */

#define BRAKE_DIST 20
#define STILL_DIST 0
#define STRAIGHT 0
#define LEFT -1
#define RIGHT 1
#define STOP_VEL 0
#define SLOW_VEL 0.5
#define FULL_VEL 0.7

/* initial positions */

#define BEFORE_STOP 0
#define AFTER_STOP 1

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
    /* skip mission when passing stopline */
    if (s->pos >= AFTER_STOP) {
        return true;
    }

    return false;
}

bool cmd_stop(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    if (s->pos == BEFORE_STOP && s->stop_visible) {
        c->vel.value = -1;
    } else if (s->pos >= AFTER_STOP) {
        c->vel.value = 0;
        if (s->last_cmd || s->postime >= PICKUP_TIME) {
            return true;
        }
    }

    return false;
}

#define PARKED AFTER_STOP+1
#define UNPARKING PARKED+1

bool cmd_park(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    switch (s->pos) {
        case BEFORE_STOP:
            if (s->stop_visible)
                i->ignore_left = true;
            break;
        case AFTER_STOP:
            c->vel.value = SLOW_VEL;
            i->ignore_left = true;
            if (s->posdist < 0.3) {
                c->rot.value = RIGHT;
            } else if (s->posdist > 0.8) {
                if (s->last_cmd || s->postime >= PICKUP_TIME) {
                    return true;
                } else {
                    s->pos = PARKED;
                }
            }
            break;
        case PARKED:
            c->vel.value = 0;
            if (s->postime >= PICKUP_TIME)
                s->pos = UNPARKING;
            break;
        case UNPARKING:
            i->ignore_left = true;
            if (s->posdist < 0.2) {
                c->rot.value = LEFT;
            } else if (s->posdist > 1) {
                return true;
            }
            break;
    }
    return false;
}

bool cmd_enter(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    if (s->pos >= AFTER_STOP) {
        if (s->posdist < 0.1) {
            i->ignore_left = true;
            return false;
        } else {
            return true;
        }
    }
    return false;
}

bool cmd_continue(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    if (s->pos == BEFORE_STOP && s->stop_visible) {
        i->ignore_right = true;
    }

    if (s->pos >= AFTER_STOP) {
        if (s->posdist < 0.7) {
            i->ignore_right = true;
            return false;
        } else {
            return true;
        }
    }

    return false;
}

bool cmd_exit(struct state *s, struct ctrl_val *c, struct ip_opt *i) {
    if (s->pos >= AFTER_STOP) {
        if (s->posdist < 1) {
            i->ignore_left = true;
            return false;
        } else {
            return true;
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

    /* current command state */
    double passtime; /* time when passing stopline or changing pos */
    double passdist;
    int pos;

    pthread_mutex_t lock;
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
    obj->ip = ip;
    obj_set_mission(obj, 0, NULL);

    return obj;
}

void obj_destroy(struct obj *obj) {
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
        obj->current = NULL;
        obj->queue = queue;
        obj->passtime = 0;
        obj->passdist = 0;
        obj->pos = BEFORE_STOP;
#ifdef IP
        ip_reset(obj->ip);
#endif
        pthread_mutex_unlock(&obj->lock);
        return true;
    } else {
        return false;
    }
}

/* execute objective command */
void obj_execute(struct obj *o, const struct sens_val *sens,
        struct ctrl_val *ctrl, struct ip_res *ip_save) {
    bool finished = false;

    pthread_mutex_lock(&o->lock);

    /* get current command */
    if (!o->current && o->queue) {
        o->current = o->queue->cmd;
        o->queue = o->queue->next;
    }

    struct ip_res ip_res;
#ifdef IP
    struct ip_osd *osd = NULL;
#ifdef PLOT
    struct ip_osd osd_struct = {
        .cmd = (o->current ? o->current->name : "none"),
        .pos = o->pos,
        .postime = sens->time - o->passtime,
        .posdist = sens->distance - o->passdist,
    };
    osd = &osd_struct;
#endif
    ip_process(o->ip, &ip_res, osd);
#endif

    if (ip_res.stopline_passed) {
        o->pos++;
        o->passtime = sens->time;
        o->passdist = sens->distance;
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
        state.last_cmd = !o->queue;
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
            o->pos = BEFORE_STOP;
            o->current = NULL;
        }

        finished = cmd_finished && !o->queue;
    } else {
        finished = true;
        ctrl->vel.value = 0;
    }

    if (finished) {
       o->active = false;
    }

    if (ip_save) {
        *ip_save = ip_res;
    }

    pthread_mutex_unlock(&o->lock);
}
