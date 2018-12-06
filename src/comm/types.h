#ifndef types_h
#define types_h

struct sens_val {
    float dist_front;
    float dist_right;
    float distance;
    float velocity;
    double time; /* monotonic seconds, accuracy <= nanosecond */
};

struct ctrl_val {
    struct {float value; float regulate;} vel;
    struct {float value; float regulate;} rot;
};

#endif
