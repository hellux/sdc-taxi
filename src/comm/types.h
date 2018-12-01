#ifndef types_h
#define types_h

struct sens_val {
    float dist_front;
    float dist_right;
    float distance;
    float velocity;
    long unsigned time; /* milliseconds since program start */
};

struct ctrl_val {
    struct {float value; float regulate;} vel;
    struct {float value; float regulate;} rot;
};

#endif
