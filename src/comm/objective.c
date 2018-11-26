#include "objective.h"

#include <string.h>

#include "main.h"

#define break_dist 40
#define still_dist 0
#define LEFT -100
#define RIGHT 100

bool obj_ignore(struct obj_args *args);
bool obj_stop(struct obj_args *args);
bool obj_park(struct obj_args *args);
bool obj_enter(struct obj_args *args);
bool obj_exit(struct obj_args *args);

const struct obj OBJS[] = {
    {"ignr", obj_ignore},
    {"stop", obj_stop},
    {"park", obj_park},
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

void slow_down(struct obj_args *args){
    args->override_vel = true;
    args->val->vel = 0.9 * args->sens->velocity;
}

void steer_dir(struct obj_args *args, const int dir){
    args->override_rot = true;
    if(dir == LEFT){
        args->val->rot = -100;
    }else if(dir == RIGHT){
        args->val->rot = 100;   
    }
}

bool obj_ignore(struct obj_args *args) {
    sens_dist_t stop_dist = args->ip->stopline_dist;
    if(stop_dist < still_dist){
        return true;
    }
    return false;
}

bool obj_stop(struct obj_args *args){
    sens_dist_t stop_dist = args->ip->stopline_dist;
    if(stop_dist < still_dist){
        args->val->vel = 0;
        return true;
    } 
    else if(stop_dist < break_dist){ 
        slow_down(args);
    }
    return false;
}

bool obj_park(struct obj_args *args){
    sens_dist_t stop_dist = args->ip->stopline_dist; 
    if(stop_dist < still_dist){
        steer_dir(args, RIGHT);
        //Wait until car is 45 degrees somehow (time?)
        steer_dir(args, LEFT);
        return true;
    } 
    else if(stop_dist < break_dist){ 
        slow_down(args);
    }
    return false;
}

bool obj_enter(struct obj_args *args){ 
    sens_dist_t stop_dist = args->ip->stopline_dist; 
    if(stop_dist < still_dist){
        steer_dir(args, RIGHT);
        return true;
    } 
    else if(stop_dist < break_dist){ 
        slow_down(args);
    }
    return false;
}

bool obj_exit(struct obj_args *args){
    sens_dist_t stop_dist = args->ip->stopline_dist;
    if(stop_dist < still_dist){
        steer_dir(args, RIGHT);
        return true;
    } 
    else if(stop_dist < break_dist){ 
        //slow_down(args);
    }
    return false;
}
