#include "server.h"

#include <stdlib.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
struct server {
    int socket;
};

srv_t *srv_create(const char *addr, int port){
    srv_t *server = calloc(1, sizeof(struct server));
    server->socket = socket(AF_INET, SOCK_STREAM, 0);
    fcntl(server->socket, F_SETFL, O_NONBLOCK); 
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    inet_pton(AF_INET, addr, &address.sin_addr);

    bind(server->socket, (struct sockaddr*)&address, sizeof(address));
    listen(server->socket, 100);
    
    return server;
}

void srv_destroy(srv_t *server) {
    printf("förstör\n");
    free(server);
}

void srv_listen(srv_t *server) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(server->socket,&set);
    struct timeval timeout = {0};
    select(&server->socket,&set,NULL,NULL,&timeout);

    struct sockaddr_storage sout;
    socklen_t addrlen;
    int socket = accept(server->socket, (struct sockaddr*)&sout, &addrlen);
    char buf[101];
    int size = recv(socket, buf, 100, MSG_DONTWAIT);
    if (size > 0) {
        buf[size] = 0;
        printf("%d, \"%s\"\n", size, buf );
    }
}
