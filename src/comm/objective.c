#include "objective.h"

#include <string.h>

#include "main.h"

#define brake_dist 40 //distance from line when braking starts          (cm)
#define still_dist 0  //distance from line when car should be still     (cm)
#define LEFT -100
#define RIGHT 100
#define stop_vel 0
#define slow_vel 40
#define full_vel 100

bool obj_ignore(struct car_state *state, struct data_ctrl *ctrl, void *data);
bool obj_stop(struct car_state *state, struct data_ctrl *ctrl, void *data);
bool obj_park(struct car_state *state, struct data_ctrl *ctrl, void *data);
bool obj_unpark(struct car_state *state, struct data_ctrl *ctrl, void *data);
bool obj_enter(struct car_state *state, struct data_ctrl *ctrl, void *data);
bool obj_exit(struct car_state *state, struct data_ctrl *ctrl, void *data);

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

bool obj_ignore(struct car_state *state, struct data_ctrl *ctrl, void *data) {
    sens_dist_t stop_dist = state->ip->stopline_dist;
    if (stop_dist <= still_dist) {
        return true;
    }
    return false;
}

bool obj_stop(struct car_state *state, struct data_ctrl *ctrl, void *data) {
    sens_dist_t stop_dist = state->ip->stopline_dist;
    float cur_vel = state->sens->velocity;
    if (cur_vel == 0) {
        return true;
    } else if (stop_dist <= brake_dist) {
        ctrl->vel.value = wtd_speed(stop_dist, cur_vel, stop_vel);
    }
    return false;
}

bool obj_park(struct car_state *state, struct data_ctrl *ctrl, void *data) {
    sens_dist_t stop_dist = state->ip->stopline_dist; 
    float cur_vel = state->sens->velocity;
    if (stop_dist <= still_dist) {
        ctrl->rot.value = 1;
        //Wait until car is 45 degrees somehow (time?)
        //Wait until straight
        ctrl->vel.value = wtd_speed(stop_dist, cur_vel, stop_vel);
        return true;
    } else if (stop_dist <= brake_dist) {
        ctrl->vel.value = wtd_speed(stop_dist, cur_vel, slow_vel);
    }
    return false;
}

bool obj_unpark(struct car_state *state, struct data_ctrl *ctrl, void *data) {
    sens_dist_t stop_dist = state->ip->stopline_dist; 
    float cur_vel = state->sens->velocity;
    ctrl->rot.value = -1;
    //Wait until car is 45 degrees somehow (time?)
    //Wait until straight
    ctrl->vel.value = wtd_speed(stop_dist, cur_vel, full_vel);
    return true;
}

bool obj_enter(struct car_state *state, struct data_ctrl *ctrl, void *data) {
    sens_dist_t stop_dist = state->ip->stopline_dist; 
    float cur_vel = state->sens->velocity;
    if (stop_dist <= still_dist) {
        ctrl->rot.value = 1;
        //Wait for some time
        return true;
    } else if (stop_dist <= brake_dist) {
        ctrl->vel.value = wtd_speed(stop_dist, cur_vel, slow_vel);
    }
    return false;
}

bool obj_exit(struct car_state *state, struct data_ctrl *ctrl, void *data) {
    sens_dist_t stop_dist = state->ip->stopline_dist;
    float cur_vel = state->sens->velocity;
    if (stop_dist <= still_dist) {
        ctrl->rot.value = 1;
        //Wait for some time
        return true;
    } else if (stop_dist <= brake_dist) {
        ctrl->vel.value = wtd_speed(stop_dist, cur_vel, slow_vel);
    }
    return false;
}
