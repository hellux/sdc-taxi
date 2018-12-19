#ifndef types_h
#define types_h

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

struct sens_val {
    float dist_front;
    float dist_right;
    float distance;
    float velocity;
    float acceleration;
    double time; /* monotonic seconds, accuracy <= nanosecond */
};

struct ctrl_val {
    float vel;
    float rot;
};

#endif
