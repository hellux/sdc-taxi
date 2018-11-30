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

#define SEC 1e3 /* milliseconds per second */

struct state {
    const struct sens_values *sens; 

    float lane_offset;
    bool lane_found;

    float stopline_dist; /* meters */
    unsigned stopline_since; /* milliseconds */
    bool stopline_passed;
};

struct obj {
    const char name[5];
    bool (*func)(struct state *s, struct data_ctrl *c);
};

bool obj_ignore(struct state *s, struct data_ctrl *c);
bool obj_stop(struct state *s, struct data_ctrl *c);
bool obj_park(struct state *s, struct data_ctrl *c);
bool obj_unpark(struct state *s, struct data_ctrl *c);
bool obj_enter(struct state *s, struct data_ctrl *c);
bool obj_exit(struct state *s, struct data_ctrl *c);

#define OBJ_IGNORE "ignr"
#define OBJ_STOP   "stop"
#define OBJ_PARK   "park"
#define OBJ_UNPARK "uprk"
#define OBJ_ENTER  "entr"
#define OBJ_EXIT   "exit"

const struct obj OBJS[] = {
    {OBJ_IGNORE, obj_ignore},
    {OBJ_STOP,   obj_stop},
    {OBJ_PARK,   obj_park},
    {OBJ_UNPARK, obj_unpark},
    {OBJ_ENTER,  obj_enter},
    {OBJ_EXIT,   obj_exit},
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
    return (target-current)/distance;
}

struct obj_data {
    struct timespec stopline_passtime;
    bool stopline_passed;
};

/* execute objective function */
bool obj_execute(const struct obj *obj, struct sens_values *sens,
                 float stopline_dist, float lane_offset, bool lane_found,
                 struct data_ctrl *ctrl, void **obj_data) {
    struct obj_data *o = NULL;
    if (!(*obj_data)) {
        o = calloc(1, sizeof(*o));
        *obj_data = o;
    }

    struct state state;

    if (stopline_dist <= 0) {
        clock_gettime(CLOCK_MONOTONIC, &o->stopline_passtime);
        o->stopline_passed = true;
    }

    unsigned since = 0;
    if (o->stopline_passed) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        time_t ds = now.tv_sec - o->stopline_passtime.tv_sec;
        long dns = now.tv_nsec - o->stopline_passtime.tv_nsec;

        since = ds*1e3 + dns/1e6;
    }

    state.sens = sens;

    state.lane_offset = lane_offset;
    state.lane_found = lane_found;

    state.stopline_dist = stopline_dist;
    state.stopline_since = since;
    state.stopline_passed = o->stopline_passed;

    return obj->func(&state, ctrl);
}

/* objective functions */

bool obj_ignore(struct state *s, struct data_ctrl *c) {
    if (s->stopline_passed) {
        return true;
    }
    return false;
}

bool obj_stop(struct state *s, struct data_ctrl *c) {
    float cur_vel = s->sens->velocity;
    if (cur_vel == STOP_VEL)
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

bool obj_unpark(struct state *s, struct data_ctrl *c) {
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
            c->vel.value = FULL_VEL;
            c->rot.value = STRAIGHT;
        }
    }     
    return false;
}
