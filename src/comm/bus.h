/* handle communication with sensor and control module via hardware bus */

#include <stdint.h>

typedef struct bus bus_t;

typedef bus_order_t bus_order;

/* allocate resources and start bus thread */
bus_t *bus_create(int freq);

/* deallocate resources and terminate thread, may block */
void bus_destroy(bus_t *bus);

/* schedule a transmit, block until finished */
void bus_transmit(bus_t *bus, int slave, uint8_t cmd, char *data, int len);

/* schedule a transmit, signal will be called when finished */
void bus_transmit_schedule(bus_t *bus, int slave, uint8_t cmd, char *data, int len,
                          void *data, void (*signal)(void *data));

/* schedule a receive, block until finished */
void bus_receive(bus_t *bus, int slave, uint8_t cmd, char *data, int len);

/* schedule a receive, signal will be called when finished */
void bus_receive_schedule(bus_t *bus, int slave, uint8_t cmd, char *data, int len,
                          void *data, void (*signal)(void *data));
