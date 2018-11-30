#include "bus.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <pthread.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define SPI_DEVICE "/dev/spidev0."

#define WAITTIME 1 /* seconds before wake up if nothing scheduled */

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

static void spi_tranceive(int fd, void *src, void *dst, int len) {
    struct spi_ioc_transfer transfer = {0};
    transfer.tx_buf = (intptr_t)src;
    transfer.rx_buf = (intptr_t)dst;
    transfer.len = (uint32_t)len;

    ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
}

static void spi_sync(int fd) {
    int cmd = BB_INVALID;
    for (int i = 0; i < MAX_DATA_LENGTH+2; i++) {
        spi_tranceive(fd, (void*)&cmd, NULL, 1);
    }
}

/* physical bus functions (bus thread) */
static bool receive(int fd, const struct bus_cmd *bc, void *dst) {
    bool success = false;
#ifdef PI
    cs_t cs = cs_create(bc->cmd, NULL, 0);
    spi_tranceive(fd, (void*)&cs, NULL, 1);
    spi_tranceive(fd, NULL, dst, bc->len);
    success = cs_check(cs, dst, bc->len);
#else
    success = true;
#endif
    return success;
}

static bool transmit(int fd, const struct bus_cmd *bc, void *msg) {
    bool success = false;
    /*
    printf("transmit:\n");
    for (int i = 0; i < bc->len; i++)
        printf("%02x ", ((uint8_t*)msg)[i]);
    printf("\n");
    */
    cs_t cs = cs_create(bc->cmd, msg, bc->len);
#ifdef PI
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
            bool success = false;
            int fd = bus->fds[order->bc->slave];
            if (order->bc->write) {
                success = transmit(fd, order->bc, order->src_dst);
            } else {
                success = receive(fd, order->bc, order->src_dst);
            }

            packets_sent++;

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
                packets_lost++;
                printf("packets lost: %d, packet loss: %.1f\n", packets_lost,
                    ((float)packets_lost/(float)packets_sent)*100);
                spi_sync(fd);
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

    /* setup spi for each slave */
#ifdef PI
    bus->fds[0] = open(SPI_DEVICE "0", O_RDWR);
    bus->fds[1] = open(SPI_DEVICE "1", O_RDWR);
    for (int i = 0; i < 2; i++) {
        uint8_t mode = SPI_MODE_0;
        uint8_t bpw = 8;
        ioctl(bus->fds[i], SPI_IOC_WR_MODE, &mode);
        ioctl(bus->fds[i], SPI_IOC_WR_BITS_PER_WORD, &bpw);
        ioctl(bus->fds[i], SPI_IOC_WR_MAX_SPEED_HZ, &freq);
    }
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
    if (bus->fds[0] >= 0) close(bus->fds[0]);
    if (bus->fds[1] >= 0) close(bus->fds[1]);
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
