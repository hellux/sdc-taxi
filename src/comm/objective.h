#include <stdbool.h>

#include "../spi/protocol.h"
#include "ip/img_proc.h"

struct obj_res {
    ctrl_val_t vel;
    ctrl_val_t rot;
    bool override_vel;
    bool override_rot;
};

struct obj {
    char *name;
    bool (*func)(const struct ip_res *ip,
                 const struct sens_data *sens,
                 struct obj_res *res);
};

struct obj_item {
    struct obj *obj;
    struct obj_item *next;
};
