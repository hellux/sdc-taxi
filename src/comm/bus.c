#include "bus.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include <pthread.h>

#include "spi.h"

#define SPI_DEVICE "/dev/spidev0."

#define WAIT_SLEEP 1 /* seconds before wake up if nothing scheduled */
#define WAIT_DELAY 20e3 /* nanoseconds between orders */
#define RECV_DELAY 20e3 /* nanoseconds between orders */

static unsigned packets_sent = 0;
static unsigned packets_lost = 0;

struct bus {
    pthread_t thread;

    pthread_cond_t wake_up;
    pthread_mutex_t wake_up_mutex;

    int fds[2];

    struct order *queue;
    bool terminate;
    pthread_mutex_t lock;
};

struct order {
    const struct bus_cmd *bc;
    void *src_dst;
    bool recurring;

    void (*handler)(void *src_dst, void *data);
    void *handler_data;

    struct order *next;
};

static bool receive(int fd, const struct bus_cmd *bc, void *dst) {
    bool success = false;
#ifdef PI
    cs_t cs = cs_create(bc->cmd, NULL, 0);
    cs_t cs_recv;
    uint8_t ack;
    spi_tranceive(fd, (void*)&cs, NULL, 1);
    spi_tranceive(fd, NULL, (void*)&ack, 1);
    if (ack == ACKS[bc->slave]) {
        spi_tranceive(fd, NULL, (void*)&cs_recv, 1);
        spi_tranceive(fd, NULL, dst, bc->len);
        success = cs_check(cs_recv, dst, bc->len);
    } else {
        success = false;
    }
#else
    success = true;
#endif
    return success;
}

static bool transmit(int fd, const struct bus_cmd *bc, void *msg) {
    bool success = false;
#ifdef PI
    cs_t cs = cs_create(bc->cmd, msg, bc->len);
    spi_tranceive(fd, (void*)&cs, NULL, 1);

    if (bc->len > 0)
        spi_tranceive(fd, (void*)msg, NULL, bc->len);

    uint8_t ack = 0;
    spi_tranceive(fd, NULL, (void*)&ack, 1);

    success = (ack == ACKS[bc->slave]);
#else
    success = true;
#endif

    return success;
}

/* order functions */

static void order_destroy(struct order *o) {
    free(o->src_dst);
    free(o);
}

/* -if not a requeue, the order will replace the previous order with the same
 *  type of command */
static void order_queue(struct bus *bus, struct order *o, bool requeue) {
    o->next = NULL;

    pthread_mutex_lock(&bus->lock);
    struct order **prev_ptr = &bus->queue;
    struct order *curr = bus->queue;
    bool insert = true;
    if (curr) {
        while (curr) {
            if (o->bc == curr->bc) {
                if (requeue) {
                    insert = false;
                    order_destroy(o);
                }
                break;
            } else {
                prev_ptr = &curr->next;
                curr = curr->next;
            }
        }
    }
    if (insert) {
        o->next = (curr) ? curr->next : NULL;
        *prev_ptr = o;
        if (curr) {
            order_destroy(curr);
        }
    }
    pthread_mutex_unlock(&bus->lock);
}

/* bus thread function */

static void *bus_thread(void *b) {
    struct bus *bus = (struct bus*)b;

    bool quit = false;

    struct timespec ts_sleep;
    struct timespec ts_delay = {0, WAIT_DELAY};

    while (!quit) {
        /* get first order in queue, if any */
        struct order *order;
        pthread_mutex_lock(&bus->lock);
        order = bus->queue;
        if (order)
            bus->queue = order->next;
        quit = bus->terminate;
        pthread_mutex_unlock(&bus->lock);

        /* execute order if any, otherwise go to sleep */
        if (order) {
            bool success = false;
            int fd = bus->fds[order->bc->slave];
            if (order->bc->write) {
                success = transmit(fd, order->bc, order->src_dst);
            } else {
                success = receive(fd, order->bc, order->src_dst);
            }

            packets_sent++;

            if (success) {
                if (order->handler)
                    order->handler(order->src_dst, order->handler_data);
                if (order->recurring) {
                    order_queue(bus, order, true);
                } else {
                    order_destroy(order);
                }
            } else {
                packets_lost++;
                printf("slave: %d, cmd: %01x, packets lost: %d, packet loss: %.1f\n",
                       order->bc->slave, order->bc->cmd, packets_lost,
                    ((float)packets_lost/(float)packets_sent)*100);
                order_queue(bus, order, true);
                spi_sync(fd, MAX_DATA_LENGTH+2);
            }

            nanosleep(&ts_delay, NULL);
        } else {
            pthread_mutex_lock(&bus->wake_up_mutex);
            clock_gettime(CLOCK_REALTIME, &ts_sleep);
            ts_sleep.tv_sec += WAIT_SLEEP;
            /* use timedwait to prevent deadlocks */
            pthread_cond_timedwait(&bus->wake_up, &bus->wake_up_mutex,
                                   &ts_sleep);
            pthread_mutex_unlock(&bus->wake_up_mutex);
        }
    }

    pthread_exit(NULL);
}

/* external API functions */

struct bus *bus_create(int freq) {
    struct bus *bus = calloc(1, sizeof(struct bus));
    bus->terminate = false;
    bus->queue = NULL;
    
    /* init synchronization */
    pthread_mutex_init(&bus->lock, NULL);
    pthread_cond_init(&bus->wake_up, NULL);
    pthread_mutex_init(&bus->wake_up_mutex, NULL);

#ifdef PI
    /* setup spi for each slave */
    bus->fds[0] = spi_create(SPI_DEVICE "0", freq);
    bus->fds[1] = spi_create(SPI_DEVICE "1", freq);
#endif

    /* start bus thread */
    pthread_create(&bus->thread, NULL, *bus_thread, (void*)(bus));

    return bus;
}

void bus_destroy(struct bus *bus) {
    /* schedule bus thread for termination */
    pthread_mutex_lock(&bus->lock);
    bus->terminate = true;
    pthread_mutex_unlock(&bus->lock);

    /* wake up bus thread if sleeping */
    pthread_cond_signal(&bus->wake_up);

    /* block until thread terminated */
    pthread_join(bus->thread, NULL);

    /* free resources */
    struct order *current = bus->queue;
    spi_destroy(bus->fds[0]);
    spi_destroy(bus->fds[1]);
    while (current) {
        struct order *prev = current;
        current = current->next;
        order_destroy(prev);
    }
    pthread_mutex_destroy(&bus->lock);
    pthread_cond_destroy(&bus->wake_up);
    pthread_mutex_destroy(&bus->wake_up_mutex);
    free(bus);
}

void bus_sync(struct bus *bus) {
    spi_sync(bus->fds[SLAVE_SENS], MAX_DATA_LENGTH+2);
    spi_sync(bus->fds[SLAVE_CTRL], MAX_DATA_LENGTH+2);
}

void bus_tranceive(struct bus *bus, const struct bus_cmd *bc, void *msg) {
    /*bus_schedule(bus, bc, msg, NULL, NULL);*/
}

void bus_schedule(struct bus *bus, const struct bus_cmd *bc, void *msg,
                  void (*handler)(void *src, void *data),
                  void *handler_data,
                  bool recurring) {
    struct order *order = malloc(sizeof(*order));
    order->bc = bc;
    order->recurring = recurring;

    order->src_dst = malloc(bc->len);
    if (bc->write) {
        memcpy(order->src_dst, msg, bc->len);
    }

    order->handler = handler;
    order->handler_data = handler_data;

    order_queue(bus, (struct order*)order, false);
    
    pthread_cond_signal(&bus->wake_up);
}
