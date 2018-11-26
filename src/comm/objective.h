#include <stdbool.h>

#include "../spi/protocol.h"
#include "ip/img_proc.h"
#include "main.h"


struct obj_args {
    struct ip_res *ip;
    struct ctrl_pair *val;
    bool override_vel;
    bool override_rot;
    bool on_road;
    struct sens_values *sens; 
};

struct obj {
    char *name;
    bool (*func)(struct obj_args);
};

struct obj_item {
    struct obj *obj;
    struct obj_item *next;
};
