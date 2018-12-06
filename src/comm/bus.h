#ifndef bus_h
#define bus_h

#include <stdint.h>

#include "protocol.h"

typedef struct bus bus_t;

/* allocate resources and start bus thread */
bus_t *bus_create(int freq);

/* deallocate resources and terminate thread, blocks until dead */
void bus_destroy(bus_t *bus);

/* ensure modules are in sync */
void bus_sync(bus_t *bus);

/* transmit or receive and block until finished */
void bus_tranceive(struct bus *bus, const struct bus_cmd *bc, void *msg);

/* schedule a receive or transmit
 * signal handler, if not NULL, will be called when finished */
/* input data is copied and can thus be freed immediately after call */
void bus_schedule(bus_t *bus, const struct bus_cmd *bc, void *msg,
                           void (*handler)(void *src, void *data),
                           void *handler_data);

#endif
