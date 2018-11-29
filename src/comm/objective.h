#ifndef objective_h
#define objective_h

#include <stdbool.h>

#include "protocol.h"
#include "ip/img_proc.h"

struct obj_args {
    const struct car_state *state;
    struct data_ctrl *ctrl;

    void *data;
};

struct obj {
    char name[5];
    bool (*func)(struct car_state*, struct data_ctrl*, void *data);
};

struct obj_item {
    const struct obj *obj;
    struct obj_item *next;
};

struct obj_item *objq_create(int cmdc, char **cmds);
void objq_destroy(struct obj_item *queue);

#endif
