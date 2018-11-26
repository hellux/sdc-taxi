#include "objective.h"

#include <string.h>

#include "main.h"

#define break_dist 40
#define still_dist 0
#define LEFT -100
#define RIGHT 100
#define stop_vel 0
#define slow_vel 40
#define full_vel 100

bool obj_ignore(struct obj_args *args);
bool obj_stop(struct obj_args *args);
bool obj_park(struct obj_args *args);
bool obj_unpark(struct obj_args *args);
bool obj_enter(struct obj_args *args);
bool obj_exit(struct obj_args *args);

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
    //TODO Calculate smooth transition from current to desired speed
    float new_vel = 0;
    return new_vel;
}

void steer_dir(struct obj_args *args, int dir){
    args->override_rot = true;
    if (dir == LEFT) {
        args->val->rot = -100;
    } else if (dir == RIGHT) {
        args->val->rot = 100;   
    }
}

bool obj_ignore(struct obj_args *args) {
    sens_dist_t stop_dist = args->ip->stopline_dist;
    if (stop_dist <= still_dist) {
        return true;
    }
    return false;
}

bool obj_stop(struct obj_args *args){
    sens_dist_t stop_dist = args->ip->stopline_dist;
    float cur_vel = args->sens->velocity;
    if (cur_vel == 0) {
        return true;
    } else if (stop_dist <= break_dist) {
        args->override_vel = true;
        args->val->vel = wtd_speed(stop_dist, cur_vel, stop_vel);
    }
    return false;
}

bool obj_park(struct obj_args *args){
    sens_dist_t stop_dist = args->ip->stopline_dist; 
    float cur_vel = args->sens->velocity;
    if (stop_dist <= still_dist) {
        steer_dir(args, RIGHT);
        //Wait until car is 45 degrees somehow (time?)
        steer_dir(args, LEFT);
        //Wait until straight
        args->val->vel = wtd_speed(stop_dist, cur_vel, stop_vel);
        args->on_road = false;
        return true;
    } else if (stop_dist <= break_dist) {
        args->override_vel = true;
        args->val->vel = wtd_speed(stop_dist, cur_vel, slow_vel);
    }
    return false;
}

bool obj_unpark(struct obj_args *args){
    sens_dist_t stop_dist = args->ip->stopline_dist; 
    float cur_vel = args->sens->velocity;
    if(!(args->on_road)){
        steer_dir(args, LEFT);
        //Wait until car is 45 degrees somehow (time?)
        steer_dir(args, RIGHT);
        //Wait until straight
        args->override_vel = true;
        args->val->vel = wtd_speed(stop_dist, cur_vel, full_vel);
        args->on_road = true;
        return false;
    }
    return true;
}

bool obj_enter(struct obj_args *args) { 
    sens_dist_t stop_dist = args->ip->stopline_dist; 
    float cur_vel = args->sens->velocity;
    if (stop_dist <= still_dist) {
        steer_dir(args, RIGHT);
        //Wait for some time
        return true;
    } else if (stop_dist <= break_dist) {
        args->override_vel = true;
        args->val->vel = wtd_speed(stop_dist, cur_vel, slow_vel);
    }
    return false;
}

bool obj_exit(struct obj_args *args) {
    sens_dist_t stop_dist = args->ip->stopline_dist;
    float cur_vel = args->sens->velocity;
    if (stop_dist <= still_dist) {
        steer_dir(args, RIGHT);
        //Wait for some time
        return true;
    } else if (stop_dist <= break_dist) {
        args->override_vel = true;
        args->val->vel = wtd_speed(stop_dist, cur_vel, slow_vel);
    }
    return false;
}
