#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include <pthread.h>

#include "bus.h"
#include "server.h"
#include "img_proc.h"

void *bus_loop(void *quit) {

    while (!*(bool*)quit) {
        bus_run();
    }
    pthread_exit(NULL);
}

int main(int argc, char* args[]) {
    bool quit = false;

    pthread_t thread_bus;
    pthread_create(&thread_bus, NULL, bus_loop, (void*)(&quit));

    srv_init();
    ip_process();

    while (!quit) {
        char c = getchar();
        if (c == 'q') quit = true;
        printf("hej från main\n");
    }

    pthread_exit(NULL);
    return EXIT_SUCCESS;
}
