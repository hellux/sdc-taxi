struct ip_res {
    float stopline_dist;
    float error;
    bool error_valid;
};

void ip_init(void);
struct ip *ip_process(void);
