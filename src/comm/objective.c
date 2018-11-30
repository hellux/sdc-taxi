#include "objective.h"

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>

#include "main.h"

#define BRAKE_DIST 40 //distance from line when braking starts          (cm)
#define STILL_DIST 0  //distance from line when car should be still     (cm)
#define STRAIGHT 0
#define LEFT -1
#define RIGHT 1
#define STOP_VEL 0
#define SLOW_VEL 40
#define FULL_VEL 100

#define SEC 1000000

struct state {
    const struct sens_values *sens; 

    float lane_offset;
    bool lane_found;

    float stopline_dist;
    long long unsigned stopline_since;
    bool stopline_passed;

    bool in_roundabout;
};

struct obj {
    char name[5];
    bool (*func)(struct state *s, struct data_ctrl *c);
};

bool obj_ignore(struct state *s, struct data_ctrl *c);
bool obj_stop(struct state *s, struct data_ctrl *c);
bool obj_park(struct state *s, struct data_ctrl *c);
bool obj_unpark(struct state *s, struct data_ctrl *c);
bool obj_enter(struct state *s, struct data_ctrl *c);
bool obj_exit(struct state *s, struct data_ctrl *c);

const struct obj OBJS[] = {
    {"ignr", obj_ignore},
    {"stop", obj_stop},
    {"park", obj_park},
    {"uprk", obj_unpark},
    {"entr", obj_enter},
    {"exit", obj_exit},
};
const int OBJC = sizeof(OBJS)/sizeof(*OBJS);

struct obj_item *objq_create(int cmdc, char **cmds) {
    bool valid = true;
    struct obj_item *root = NULL;
    struct obj_item **prev_ptr = &root;
    struct obj_item *current;
    for (int i = 0; i < cmdc; i++) {
        current = calloc(1, sizeof(*current));
        *prev_ptr = current;

        for (int j = 0; j < OBJC; j++) {
            if (strcmp(cmds[i], OBJS[j].name) == 0) {
                current->obj = &OBJS[j];
                break;
            }
        }
        if (!current->obj) {
            valid = false;
            break;
        }
    }

    if (valid) {
        return root;
    } else {
        objq_destroy(root);
        return NULL;
    }

}

void objq_destroy(struct obj_item *queue) {
    while (queue) {
        struct obj_item *prev = queue;
        queue = queue->next;
        free(queue);
    }
}


ctrl_val_t wtd_speed(float distance, float current, float target) {
    //TODO update if there is no distance(no stopline)
    return (target-current)/distance;
}

bool obj_execute(struct obj *obj, struct sens_values *sens,
                 float stopline_dist, float lane_offset, bool lane_found,
                 struct data_ctrl *ctrl) {
    struct state state;

    /* TODO fill state */

    return obj->func(&state, ctrl);
}

bool obj_ignore(struct state *s, struct data_ctrl *c) {
    if (s->stopline_passed) {
        return true;
    }
    return false;
}

bool obj_stop(struct state *s, struct data_ctrl *c) {
    float cur_vel = s->sens->velocity;
    if (cur_vel == 0)
        return true;
    else if (s->stopline_dist <= BRAKE_DIST) {
        c->vel.value = wtd_speed(s->stopline_dist, cur_vel, STOP_VEL);
    }
    return false;
}

bool obj_park(struct state *s, struct data_ctrl *c) {
    float cur_vel = s->sens->velocity;
    if (s->stopline_passed) {
        c->rot.value = RIGHT;

        if (cur_vel == 0)
            return true;
        else if (s->stopline_since >= SEC){
            c->vel.value = wtd_speed(s->stopline_dist, cur_vel, STOP_VEL);
            c->rot.value =STRAIGHT;
        }
        else if (s->stopline_since >= SEC*0.5)
            c->rot.value = LEFT;
    } else if (s->stopline_dist <= BRAKE_DIST) {
        c->vel.value = wtd_speed(s->stopline_dist, cur_vel, SLOW_VEL);
    }
    return false;
}

bool obj_unpark(struct state *s, struct data_ctrl *c) {
    float cur_vel = s->sens->velocity; 
    c->rot.value = LEFT;
    c->vel.value = wtd_speed(s->stopline_dist, cur_vel, SLOW_VEL);
    
    if (cur_vel == FULL_VEL)
        return true;
    else if (s->stopline_since >= SEC){
        c->vel.value = wtd_speed(s->stopline_dist, cur_vel, FULL_VEL);
        c->rot.value = STRAIGHT;
    }
    else if (s->stopline_since >= SEC*0.5)
        c->rot.value = RIGHT;
    return false;
}

bool obj_enter(struct state *s, struct data_ctrl *c) {
    float cur_vel = s->sens->velocity;
    if (s->stopline_passed) {
        c->rot.value = RIGHT;
    
        if (s->stopline_since >= SEC*0.5){
            c->rot.value = STRAIGHT;
            s->in_roundabout = true;
            return true;
        }
    } else if (s->stopline_dist <= BRAKE_DIST) {
        c->vel.value = wtd_speed(s->stopline_dist, cur_vel, SLOW_VEL);
    }
    return false;
}

bool obj_exit(struct state *s, struct data_ctrl *c) {
    float cur_vel = s->sens->velocity;
    if (s->stopline_passed) {
        c->rot.value = RIGHT;
    
        if (cur_vel == FULL_VEL){
            s->in_roundabout = false;
            return true;
        }
        else if (s->stopline_since >= SEC*0.5){
            c->vel.value = wtd_speed(s->stopline_dist, cur_vel, FULL_VEL);
            c->rot.value = STRAIGHT;
        }
    }     
    return false;
}
