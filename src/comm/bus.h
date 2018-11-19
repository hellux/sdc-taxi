/* handle communication with modules via hardware bus */

#include <stdint.h>

typedef struct bus bus_t;

struct bus_cmd {
    uint8_t cmd;
    int slave;
    int len;
};

/* allocate resources and start bus thread */
bus_t *bus_create(int freq);

/* deallocate resources and terminate thread, blocks until dead */
void bus_destroy(bus_t *bus);

/* schedule a transmit, block until finished */
void bus_transmit(bus_t *bus, const struct bus_cmd *bc, void *msg);

/* schedule a transmit, signal handler will be called when finished */
/* input data is copied and can thus be freed immediately after call */
void bus_transmit_schedule(bus_t *bus, const struct bus_cmd *bc, void *msg,
                           void (*handler)(void *src, void *data),
                           void *handler_data);

/* schedule a receive, block until finished */
void bus_receive(bus_t *bus, const struct bus_cmd *bc, void *dst);

/* schedule a receive, signal handler will be called when finished */
void bus_receive_schedule(bus_t *bus, const struct bus_cmd *bc,
                          void (*handler)(void *dst, void *data),
                          void *handler_data);
