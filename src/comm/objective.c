#include "objective.h"
#define break_dist 40
#define still_dist 0
#define LEFT -100
#define RIGHT 100

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

