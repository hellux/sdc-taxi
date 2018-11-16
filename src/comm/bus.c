#include "bus.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <pthread.h>

#ifdef PI
#include <wiringPi.h>
#include <wiringPiSPI.h>
#endif

#define CHANNEL 0
#define SS1 10 //sens, port 10
#define SS2 11 //ctrl, port 11

struct bus {
    bus_sens_t sens;
    bus_ctrl_t ctrl;

    pthread_t thread;
    pthread_mutex_t lock;
    pthread_mutex_t idle_mutex;
    pthread_cond_t idle_cond;

    bool transmit_ctrl;
    bool receive_sens;

    bool terminate;

    int channel;
};

/* internal thread functions */

void receive_sens(bus_t *bus, bus_sens_t *data) {
#ifdef PI
    digitalWrite(SS1, 1);   // SS high - synch with slave
    digitalWrite(SS1, 0);   // SS low - start transmission
    wiringPiSPIDataRW(CHANNEL, (unsigned char*)data, sizeof(*data));
    digitalWrite(SS1, 1);   // SS high - end transmission
#endif
}

void transmit_ctrl(bus_t *bus, bus_ctrl_t *data) {
#ifdef PI
    digitalWrite(SS2, 1);   // SS high - synch with slave
    digitalWrite(SS2, 0);   // SS low - start transmission
    wiringPiSPIDataRW(CHANNEL, (unsigned char*)data, sizeof(*data));
    digitalWrite(SS2, 1);   // SS high - end transmission
#endif
}

/* separate thread for bus */
void *bus_thread(void *bus_ptr) {
    bus_t *bus = (bus_t*)bus_ptr;
    bool quit = false;

    bus_sens_t sens_local;
    bus_ctrl_t ctrl_local;

    while (!quit) {
        /* wait until woken up to transmit, receive or die */
        pthread_cond_wait(&bus->idle_cond, &bus->idle_mutex);

        bool transmit = false;
        bool receive = false;

        pthread_mutex_lock(&bus->lock);

        quit = bus->terminate;
        transmit = bus->transmit_ctrl;
        receive = bus->receive_sens;
        if (transmit) ctrl_local = bus->ctrl;

        pthread_mutex_unlock(&bus->lock);

        if (receive) {
            receive_sens(bus, &sens_local);
            pthread_mutex_lock(&bus->lock);
            bus->sens = sens_local;
            pthread_mutex_unlock(&bus->lock);
        }

        if (transmit) {
            transmit_ctrl(bus, &ctrl_local);
        }
    }

    pthread_exit(NULL);
}

/* external API functions */

bus_t *bus_create(int freq) {
    bus_t *bus = calloc(1, sizeof(struct bus));
    bus->terminate = false;
    
    /* init synchronization */
    pthread_mutex_init(&bus->lock, NULL);
    pthread_mutex_init(&bus->idle_mutex, NULL);
    pthread_cond_init(&bus->idle_cond, NULL);

    /* setup spi */
#ifdef PI
    wiringPiSPISetup(CHANNEL, freq);
#endif

    /* start bus thread */
    pthread_create(&bus->thread, NULL, bus_thread, (void*)(bus));

    return bus;
}

void bus_destroy(bus_t *bus) {
    /* schedule bus thread for termination */
    pthread_mutex_lock(&bus->lock);
    bus->terminate = true;
    bus->transmit_ctrl = false;
    bus->receive_sens = false;
    pthread_mutex_unlock(&bus->lock);

    /* wake up bus thread if sleeping */
    pthread_cond_broadcast(&bus->idle_cond);

    /* block until thread terminated */
    pthread_join(bus->thread, NULL);

    /* free resources */
    pthread_mutex_destroy(&bus->lock);
    pthread_cond_destroy(&bus->idle_cond);
    free(bus);
}

void bus_transmit_ctrl(bus_t *bus, bus_ctrl_t *data) {
    /* store new data, schedule new transmit of data */
    pthread_mutex_lock(&bus->lock);
    bus->ctrl = *data;
    bus->transmit_ctrl = true;
    pthread_mutex_unlock(&bus->lock);

    /* wake up bus thread if sleeping */
    pthread_cond_broadcast(&bus->idle_cond);
}

void bus_receive_sens(bus_t *bus) {
    pthread_mutex_lock(&bus->lock);
    bus->receive_sens = true;
    pthread_mutex_unlock(&bus->lock);

    pthread_cond_broadcast(&bus->idle_cond);
}

void bus_get_sens(bus_t *bus, bus_sens_t *data) {
    pthread_mutex_lock(&bus->lock);
    *data = bus->sens;
    pthread_mutex_unlock(&bus->lock);
}
