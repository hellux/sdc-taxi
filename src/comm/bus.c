#include "bus.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <pthread.h>

#ifdef PI
#include <wiringPi.h>
#include <wiringPiSPI.h>
#endif

#define WAITTIME 1 /* seconds before wake up if nothing scheduled */

static unsigned packets_sent = 0;
static unsigned packets_lost = 0;

struct bus {
    pthread_t thread;

    pthread_cond_t wake_up;
    pthread_mutex_t wake_up_mutex;

    struct order *queue;
    bool terminate;
    pthread_mutex_t lock;
};

struct order {
    bool scheduled;

    const struct bus_cmd *bc;
    void *src_dst;

    struct order *next;
};

struct order_scheduled {
    struct order common;

    void (*handler)(void *src_dst, void *data);
    void *handler_data;
};

struct order_blocked {
    struct order common;

    pthread_cond_t done; /* will be signaled when finished */
    pthread_mutex_t done_mutex;
};

/* physical bus functions (bus thread) */
static bool receive(const struct bus_cmd *bc, void *msg) {
    bool success = false;
#ifdef PI
    cs_t cs = cs_create(bc->cmd, NULL, 0);
    wiringPiSPIDataRW(bc->slave, (unsigned char*)&cs, 1);
    wiringPiSPIDataRW(bc->slave, (unsigned char*)msg, bc->len);
    success = cs_check(cs, msg, bc->len);
#else
    for (int i = 0; i < bc->len; i++) {
        *((uint8_t*)msg+i) = (uint8_t)i;
    }
    success = true;
#endif
    return success;
}

static bool transmit(const struct bus_cmd *bc, void *msg) {
    bool success = false;
    /*
    printf("transmit:\n");
    for (int i = 0; i < bc->len; i++)
        printf("%02x ", ((uint8_t*)msg)[i]);
    printf("\n");
    */
#ifdef PI
    cs_t cs = cs_create(bc->cmd, msg, bc->len);
    uint8_t ack;

    /* send cmd sum */
    wiringPiSPIDataRW(bc->slave, (unsigned char*)&cs, 1);

    /* send data, if any */
    if (bc->len > 0) {
        wiringPiSPIDataRW(bc->slave, (unsigned char*)msg, bc->len);
    }

    /* retrieve ack */
    wiringPiSPIDataRW(bc->slave, (unsigned char*)&ack, 1);

    success = (ack == ACKS[bc->slave]);
#else
    success = true;
#endif
    /*
    printf("received:\n");
    for (int i = 0; i < bc->len; i++)
        printf("%02x ", ((uint8_t*)msg)[i]);
    printf("\n");
    */

    return success;
}

/* order functions */

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
                if (requeue)
                    insert = false;
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
    }
    pthread_mutex_unlock(&bus->lock);
}

/* execute an order, from bus thread */
static bool order_execute(struct bus *bus, struct order *o) {
    packets_sent++;
    bool success = false;
    if (o->bc->write) {
        void *data_copy = malloc(o->bc->len);
        memcpy(data_copy, o->src_dst, o->bc->len);
        success = transmit(o->bc, data_copy);
        if (success)
            memcpy(o->src_dst, data_copy, o->bc->len);
        free(data_copy);
    } else {
        success = receive(o->bc, o->src_dst);
    }
    if (!success)
        packets_lost++;

    printf("packet loss: %.1f\n",
           ((float)packets_lost/(float)packets_sent)*100);

    return success;
}

/* bus thread function */

static void *bus_thread(void *b) {
    struct bus *bus = (struct bus*)b;

    bool quit = false;

    struct timespec ts;

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
            bool success = order_execute(bus, order);
            if (success) {
                if (order->scheduled) {
                    struct order_scheduled *os = (struct order_scheduled*)order;
                    if (os->handler)
                        os->handler(order->src_dst, os->handler_data);
                    free(order->src_dst);
                    free(order);
                } else {
                    struct order_blocked *ob = (struct order_blocked*)order;
                    pthread_cond_signal(&ob->done);
                }
            } else {
                order_queue(bus, order, true);
            }
        } else {
            pthread_mutex_lock(&bus->wake_up_mutex);
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += WAITTIME;
            /* use timedwait to prevent deadlocks */
            pthread_cond_timedwait(&bus->wake_up, &bus->wake_up_mutex, &ts);
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

    /* setup spi */
#ifdef PI
    wiringPiSetup();
    wiringPiSPISetup(0, freq);
    wiringPiSPISetup(1, freq);
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
    while (current) {
        struct order *prev = current;
        current = current->next;
        free(prev);
    }
    pthread_mutex_destroy(&bus->lock);
    pthread_cond_destroy(&bus->wake_up);
    pthread_mutex_destroy(&bus->wake_up_mutex);
    free(bus);
}

void bus_tranceive(struct bus *bus, const struct bus_cmd *bc, void *msg) {
    struct order_blocked *order = malloc(sizeof(struct order_blocked));
    order->common.scheduled = false;
    order->common.bc = bc;
    order->common.src_dst = msg;
    pthread_cond_init(&order->done, NULL);
    pthread_mutex_init(&order->done_mutex, NULL);

    order_queue(bus, (struct order*)order, false);

    pthread_mutex_lock(&order->done_mutex);
    pthread_cond_wait(&order->done, &order->done_mutex);
    pthread_mutex_unlock(&order->done_mutex);

    free(order);
}

void bus_transmit_schedule(struct bus *bus, const struct bus_cmd *bc, void *msg,
                           void (*handler)(void *src, void *data),
                           void *handler_data) {
    struct order_scheduled *order = malloc(sizeof(struct order_blocked));
    order->common.scheduled = true;
    order->common.bc = bc;

    /* copy input data */
    order->common.src_dst = malloc(bc->len);
    memcpy(order->common.src_dst, msg, bc->len);

    order->handler = handler;
    order->handler_data = handler_data;

    order_queue(bus, (struct order*)order, false);
    
    pthread_cond_signal(&bus->wake_up);
}

void bus_receive_schedule(struct bus *bus, const struct bus_cmd *bc,
                          void (*handler)(void *dst, void *data),
                          void *handler_data) {
    struct order_scheduled *order = malloc(sizeof(struct order_scheduled));
    order->common.scheduled = true;
    order->common.bc = bc;

    /* create storage for receive */
    order->common.src_dst = malloc(bc->len);

    order->handler = handler;
    order->handler_data = handler_data;

    order_queue(bus, (struct order*)order, false);
    
    pthread_cond_signal(&bus->wake_up);
}
