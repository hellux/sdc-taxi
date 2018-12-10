#ifndef objective_h
#define objective_h

#include <stdbool.h>

#include "types.h"
#include "ip/img_proc.h"

typedef struct obj obj_t;

obj_t *obj_create(void);
void obj_destroy(obj_t *obj);
bool obj_active(obj_t *obj);
void obj_set_state(obj_t *obj, bool state);
int obj_remaining(obj_t *obj);
bool obj_set_mission(obj_t *obj, int cmdc, char **cmds);
void obj_execute(obj_t *obj, const struct sens_val *sens,
                                   struct ctrl_val *ctrl,
                                   struct ip_res *ip_res);

#endif
