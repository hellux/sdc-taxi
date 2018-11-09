#include "server.h"

#include <stdlib.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/socket.h>

struct server {
    /* TODO add socket etc needed for server */
};

srv_t *srv_create(void) {
    srv_t *server = calloc(1, sizeof(struct server));
    printf("hej hej");
    /* TODO skapa server här */
    return server;
}

void srv_destroy(srv_t *server) {
    printf("förstör\n");
    free(server);
}

void srv_listen(srv_t *server) {
    printf("lyssnar\n");
    /* TODO kolla connections här */
}
