#ifndef objective_h
#define objective_h

#include <stdbool.h>

#include "main.h"
#include "protocol.h"
#include "ip/img_proc.h"

typedef struct obj obj_t;

struct obj_item {
    const obj_t *obj;
    struct obj_item *next;
};

struct obj_item *objq_create(int cmdc, char **cmds);
void objq_destroy(struct obj_item *queue);
bool obj_execute(const obj_t *obj, struct sens_values *sens,
                 float stopline_dist, float lane_offset, bool lane_found,
                 struct data_ctrl *ctrl, void **obj_data);

#endif
