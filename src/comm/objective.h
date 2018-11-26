#ifndef objective_h
#define objective_h

#include <stdbool.h>

#include "protocol.h"
#include "ip/img_proc.h"

struct obj_args {
    struct ip_res *ip;
    struct ctrl_pair *val;
    bool override_vel;
    bool override_rot;
    bool on_road;
    struct sens_values *sens; 
};

struct obj {
    char name[5];
    bool (*func)(struct obj_args *args);
};

struct obj_item {
    const struct obj *obj;
    struct obj_item *next;
};

struct obj_item *objq_create(int cmdc, char **cmds);
void objq_destroy(struct obj_item *queue);

#endif
