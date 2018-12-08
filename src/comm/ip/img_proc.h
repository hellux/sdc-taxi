#ifndef img_proc_h
#define img_proc_h

#include <stdbool.h>

struct ip_res {
    float lane_offset;
    float stopline_dist;

    bool stopline_visible;
    bool stopline_passed;
    bool lane_right_visible;
    bool lane_left_visible;
};

struct ip_opt {
    bool ignore_right;
    bool ignore_left;
    bool ignore_stop;
};

typedef struct ip_data ip_t;

#ifndef CPP
ip_t *ip_init(void);
void ip_destroy(ip_t *ip);
void ip_set_opt(ip_t *ip, struct ip_opt *opt);
void ip_process(ip_t *ip, struct ip_res *res);
void ip_reset(ip_t *ip);
#endif

#endif
