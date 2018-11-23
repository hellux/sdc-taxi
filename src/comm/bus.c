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

#define CHANNEL 0 /* channel to use on RPI */
#define WAITTIME 1 /* seconds before wake up if nothing scheduled */

#define SLAVE_SENS_GPIO 7
#define SLAVE_CTRL_GPIO 8

#ifdef PI
static int GPIO_PINS[2] = {SLAVE_SENS_GPIO, SLAVE_CTRL_GPIO};
static int ACKS[2] = {SENS_ACK, CTRL_ACK};
#endif

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
    digitalWrite(GPIO_PINS[bc->slave], 1);
    digitalWrite(GPIO_PINS[bc->slave], 0);
    wiringPiSPIDataRW(CHANNEL, (unsigned char*)&cs, 1);
    wiringPiSPIDataRW(CHANNEL, (unsigned char*)msg, bc->len);
    digitalWrite(GPIO_PINS[bc->slave], 1);
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
#ifdef PI
    cs_t cs = cs_create(bc->cmd, msg, bc->len);
    uint8_t ack = ACKS[bc->slave];
    digitalWrite(GPIO_PINS[bc->slave], 1);
    digitalWrite(GPIO_PINS[bc->slave], 0);
    wiringPiSPIDataRW(CHANNEL, (unsigned char*)&cs, 1);
    wiringPiSPIDataRW(CHANNEL, (unsigned char*)msg, bc->len);
    wiringPiSPIDataRW(CHANNEL, (unsigned char*)&ack, 1);
    digitalWrite(GPIO_PINS[bc->slave], 1);
    success = (ack == ACKS[bc->slave]);
#else
    printf("transmit via command %d to %d: ", bc->cmd, bc->slave);
    for (int i = 0; i < bc->len; i++)
        printf("%x ", ((uint8_t*)msg)[i]);
    printf("\n");
    success = true;
#endif
    return success;
}

/* order functions */

/* queue an order, from outside thread */
static void order_queue(struct bus *bus, struct order *order) {
    order->next = NULL;
    struct order *last = bus->queue;
    if (last) {
        while (last->next) last = last->next;
        last->next = order;
    } else {
        bus->queue = order;
    }
}

/* execute an order, from bus thread */
static void order_execute(struct bus *bus, struct order *o) {
    bool success = false;
    if (o->bc->write) {
        success = transmit(o->bc, o->src_dst);
    } else {
        success = receive(o->bc, o->src_dst);
    }

    if (success) {
        printf("success\n");
        if (o->scheduled) {
            struct order_scheduled *os = (struct order_scheduled*)o;
            if (os->handler)
                os->handler(o->src_dst, os->handler_data);
            free(o->src_dst);
            free(o);
        } else {
            struct order_blocked *ob = (struct order_blocked*)o;
            pthread_cond_signal(&ob->done);
        }
    } else {
        order_queue(bus, o);
    }
}

/* bus thread function */

static void *bus_thread(void *b) {
    struct bus *bus = (struct bus*)b;

    bool quit = false;

    struct timespec ts;

    while (!quit) {
        pthread_mutex_lock(&bus->lock);

        /* execute one order, allow scheduling new orders inbetween */
        struct order *order = bus->queue;
        bool empty = (bus->queue == NULL);
        if (!empty) {
            bus->queue = order->next;
            order_execute(bus, order);
        }

        quit = bus->terminate;

        pthread_mutex_unlock(&bus->lock);

        if (empty) {
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
    wiringPiSetupGpio();
    wiringPiSPISetup(CHANNEL, freq);
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

void bus_tranceive(bus_t *bus, const struct bus_cmd *bc, void *msg) {
    struct order_blocked *order = malloc(sizeof(struct order_blocked));
    order->common.scheduled = false;
    order->common.bc = bc;
    order->common.src_dst = msg;
    pthread_cond_init(&order->done, NULL);
    pthread_mutex_init(&order->done_mutex, NULL);

    pthread_mutex_lock(&bus->lock);
    order_queue(bus, (struct order*)order);
    pthread_mutex_unlock(&bus->lock);

    pthread_mutex_lock(&order->done_mutex);
    pthread_cond_wait(&order->done, &order->done_mutex);
    pthread_mutex_unlock(&order->done_mutex);

    free(order);
}

void bus_transmit_schedule(bus_t *bus, const struct bus_cmd *bc, void *msg,
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

    pthread_mutex_lock(&bus->lock);
    order_queue(bus, (struct order*)order);
    pthread_mutex_unlock(&bus->lock);
    
    pthread_cond_signal(&bus->wake_up);
}

void bus_receive_schedule(bus_t *bus, const struct bus_cmd *bc,
                          void (*handler)(void *dst, void *data),
                          void *handler_data) {
    struct order_scheduled *order = malloc(sizeof(struct order_scheduled));
    order->common.scheduled = true;
    order->common.bc = bc;

    /* create storage for receive */
    order->common.src_dst = malloc(bc->len);

    order->handler = handler;
    order->handler_data = handler_data;

    pthread_mutex_lock(&bus->lock);
    order_queue(bus, (struct order*)order);
    pthread_mutex_unlock(&bus->lock);
    
    pthread_cond_signal(&bus->wake_up);
}
