/* handle communication with modules via hardware bus */

#include <stdint.h>

typedef struct bus bus_t;

/* allocate resources and start bus thread */
bus_t *bus_create(int freq);

/* deallocate resources and terminate thread, blocks until dead */
void bus_destroy(bus_t *bus);

/* schedule a transmit, block until finished */
void bus_transmit(bus_t *bus, int slave,
                  uint8_t cmd, unsigned char *data, int len);

/* schedule a transmit, signal handler will be called when finished */
/* data will be freed when transmit finished */
void bus_transmit_schedule(bus_t *bus, int slave,
                           uint8_t cmd, unsigned char *data, int len,
                           void (*handler)(unsigned char *src, void *data),
                           void *handler_data);

/* schedule a receive, block until finished */
void bus_receive(bus_t *bus, int slave,
                 uint8_t cmd, unsigned char *dst, int len);

/* schedule a receive, signal handler will be called when finished */
void bus_receive_schedule(bus_t *bus, int slave, uint8_t cmd, int len,
                          void (*handler)(unsigned char *dst, void *data),
                          void *handler_data);
