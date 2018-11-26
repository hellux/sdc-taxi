#include "objective.h"
#define break_dist 40
#define still_dist 0
#define LEFT -100
#define RIGHT 100

#define stop_vel 0
#define slow_vel 40
#define full_vel 100


ctrl_val_t wtd_speed(const float stop_dist, const float cur_vel, const float des_vel){
    //TODO Calculate smooth transition from current to desired speed
    float new_vel= 0;
    return new_vel;
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
    if(stop_dist <= still_dist){
        return true;
    }
    return false;
}

bool obj_stop(struct obj_args *args){
    sens_dist_t stop_dist = args->ip->stopline_dist;
    float cur_vel = args->sens->velocity;
    if(cur_vel == 0){
        return true;
    } 
    else if(stop_dist <= break_dist){
        args->override_vel = true;
        args->val->vel = wtd_speed(stop_dist, cur_vel, stop_vel);
    }
    return false;
}

bool obj_park(struct obj_args *args){
    sens_dist_t stop_dist = args->ip->stopline_dist; 
    float cur_vel = args->sens->velocity;
    if(stop_dist <= still_dist){
        steer_dir(args, RIGHT);
        //Wait until car is 45 degrees somehow (time?)
        steer_dir(args, LEFT);
        //Wait until straight
        args->val->vel = wtd_speed(stop_dist, cur_vel, stop_vel);
        args->on_road = false;
        return true;
    } 
    else if(stop_dist <= break_dist){ 
        args->override_vel = true;
        args->val->vel = wtd_speed(stop_dist, cur_vel, slow_vel);
    }
    return false;
}

bool obj_un_park(struct obj_args *args){
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

bool obj_enter(struct obj_args *args){ 
    sens_dist_t stop_dist = args->ip->stopline_dist; 
    float cur_vel = args->sens->velocity;
    if(stop_dist <= still_dist){
        steer_dir(args, RIGHT);
        //Wait for some time
        return true;
    } 
    else if(stop_dist <= break_dist){ 
        args->override_vel = true;
        args->val->vel = wtd_speed(stop_dist, cur_vel, slow_vel);
    }
    return false;
}

bool obj_exit(struct obj_args *args){
    sens_dist_t stop_dist = args->ip->stopline_dist;
    float cur_vel = args->sens->velocity;
    if(stop_dist <= still_dist){
        steer_dir(args, RIGHT);
        //Wait for some time
        return true;
    } 
    else if(stop_dist <= break_dist){ 
        args->override_vel = true;
        args->val->vel = wtd_speed(stop_dist, cur_vel, slow_vel);
    }
    return false;
}

