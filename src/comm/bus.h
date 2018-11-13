/* handle communication with sensor and control module via hardware bus */

#include <stdint.h>

typedef struct bus bus_t;

typedef struct {
    uint8_t dist_front; //distance to object in front
    uint8_t dist_right; //distance to object to the right
    unsigned rotations; //wheel rotations
} bus_sens_t; 

typedef struct {
    int8_t err_vel;     //velocity error
    int8_t err_rot;     //rotation error
} bus_ctrl_t;

/* allocate resources and start bus thread */
bus_t *bus_create(int freq);

/* deallocate resources and terminate thread, may block */
void bus_destroy(bus_t *bus);

/* schedule transmission of values to control module */
void bus_transmit_ctrl(bus_t *bus, bus_ctrl_t *data);

/* get latest values from sensor module */
void bus_get_sens(bus_t *bus, bus_sens_t *data);

/* schedule receiving new values from sensor module */
void bus_receive_sens(bus_t *bus);
