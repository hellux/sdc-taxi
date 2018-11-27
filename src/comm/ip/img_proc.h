#ifndef im_proc_h
#define im_proc_h

#include <stdbool.h>

struct ip_res {
    float stopline_dist;
    bool stopline_found;
    float error;
    bool error_valid;
};

typedef struct ip_data ip_t;

#ifdef CPP
extern "C" ip_t *ip_init(void);
extern "C" void ip_destroy(ip_t *ip);
extern "C" void ip_process(ip_t *ip, struct ip_res *res);
#else
ip_t *ip_init(void);
void ip_destroy(ip_t *ip);
void ip_process(ip_t *ip, struct ip_res *res);
#endif

#endif
