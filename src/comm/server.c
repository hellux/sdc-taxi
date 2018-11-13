#include "server.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

/* starting size of buffer for received messages */
#define BUF_SIZE 100

struct server {
    int listen_fd; /* fd for socket that is listened to */
    int conn_fd; /* fd for connection, -1 if no connection */
};

/* internal functions */

void accept_connection(srv_t *srv) {
    int conn_fd = accept(srv->listen_fd, NULL, NULL);
    if (conn_fd >= 0) {
        printf("accepted new connection, overwrite previous\n");
        close(srv->conn_fd);
        srv->conn_fd = conn_fd;
    }
}

/* string returned must be freed by caller */
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

void send_data(srv_t *srv, char *data) {
    send(srv->conn_fd, data, strlen(data), 0);
}

void parse_commands(const char *str) {
    /* TODO */
}

/* external api functions */
srv_t *srv_create(const char *addr, int port){
    int succ;

    srv_t *srv = calloc(1, sizeof(srv_t));
    srv->conn_fd = -1;

    srv->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (srv->listen_fd == -1) {
        perror("failed to create unbound socket");
        goto fail;
    }

    /* ensure no blocking when accepting connections */
    fcntl(srv->listen_fd, F_SETFL, O_NONBLOCK); 

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    inet_pton(AF_INET, addr, &address.sin_addr);

    succ = bind(srv->listen_fd, (struct sockaddr*)&address, sizeof(address));
    if (succ == -1) {
        perror("failed to bind socket");
        goto fail;
    }

    succ = listen(srv->listen_fd, 10);
    if (succ == -1) {
        perror("failed to listen to socket");
        goto fail;
    }

    return srv;
fail:
    srv_destroy(srv);
    return NULL;
}

void srv_destroy(srv_t *srv) {
    if (srv->listen_fd > 0) close(srv->listen_fd);
    if (srv->conn_fd > 0) close(srv->conn_fd);
    free(srv);
}

void srv_listen(srv_t *srv) {
    accept_connection(srv);

    char *data = receive(srv);
    if (data) {
        printf("received: \"%s\"\n", data);
        int *command = parse_commands(srv, data);
        char data_re[100];

        sprintf(data_re, "ditt kommando var: %s", data);
        printf("%s, %d\n", data, strncmp("get", data, 3));
        if (!strncmp(data, "get", 3)) {
            send_data(srv, data_re);
        }
    }

    free(data);
}
