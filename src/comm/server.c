#include "server.h"

#include <stdlib.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#define BUF_SIZE 50

struct server {
    int listen_fd; /* fd for socket that is listened to */
    int conn_fd; /* fd for connection, -1 if no connection */
};

/* internal functions */

void accept_connection(srv_t *srv) {
    int conn_fd = accept(srv->listen_fd, NULL, NULL);
    if (conn_fd >= 0) {
        close(srv->conn_fd);
        srv->conn_fd = conn_fd;
    }
}

char *receive(srv_t *srv) {
    int bufsize = BUF_SIZE;
    char *buf = malloc(bufsize*sizeof(char));
    int length = 0;

    if (srv->conn_fd >= 0) {
        int max_receive;
        int received = 0;

        do {
            if (length >= bufsize) {
                bufsize *= 2;
                buf = realloc(buf, bufsize+1);
            }
            max_receive = bufsize-length;
            received = recv(srv->conn_fd, buf+length, max_receive, 0);
            length += received;
        } while (received == max_receive);
    }

    if (length == 0) {
        free(buf);
        buf = NULL;
    } else {
        buf[length] = '\0';
    }

    return buf;
}

/* external api functions */
srv_t *srv_create(const char *addr, int port){
    int succ;

    srv_t *srv = calloc(1, sizeof(srv_t));
    srv->conn_fd = -1;

    srv->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (srv->listen_fd == -1) {
        perror("failed to create unbound socket");
        return NULL;
    }
    fcntl(srv->listen_fd, F_SETFL, O_NONBLOCK); 

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    inet_pton(AF_INET, addr, &address.sin_addr);

    succ = bind(srv->listen_fd, (struct sockaddr*)&address, sizeof(address));
    if (succ == -1) {
        perror("failed to bind socket");
        return NULL;
    }

    succ = listen(srv->listen_fd, 10);
    if (succ == -1) {
        perror("failed to listen to socket");
        return NULL;
    }

    return srv;
}

void srv_destroy(srv_t *srv) {
    close(srv->listen_fd);
    close(srv->conn_fd);
    free(srv);
}

void srv_listen(srv_t *srv) {
    accept_connection(srv);

    char *data = receive(srv);
    if (data) printf("received: \"%s\"\n", data);
}
