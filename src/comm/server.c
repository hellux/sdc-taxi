#include "server.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <pthread.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#define MAX(A, B) ((A) > (B) ? (A) : (B))

#define COMM_DELIM ":"
#define ARG_DELIM ","
#define ARG_BUF_SIZE 64

/* starting size of buffer for received messages */
#define MSG_BUF_SIZE 2048

struct comm_item {
    struct srv_command command;
    char *arg_str; /* string with arguments */
    char **args; /* pointers to arguments in string */
    int argc;
    struct comm_link *next;
};

struct server {
    int listen_fd; /* fd for socket that is listened to */

    pthread_t thread;
    struct srv_command *comms;
    int commc;

    struct {
        struct comm_item queue;
        int queue_size;
        bool terminate;
    } shared;
};

/* internal functions */

/* string returned must be freed by caller */
char *receive(int conn_fd, int *msglen) {
    int bufsize = MSG_BUF_SIZE;
    char *buf = malloc(bufsize*sizeof(char));
    int length = 0;

    int max_receive;
    int received = 0;

    if (conn_fd >= 0) {
        do {
            if (length >= bufsize) {
                bufsize *= 2;
                buf = realloc(buf, bufsize+1);
            }
            max_receive = bufsize-length;
            received = recv(conn_fd, buf+length, max_receive, 0);
            length += received;
        } while (received == max_receive);
    }

    if (length == 0) {
        free(buf);
        buf = NULL;
    } else {
        buf[length] = '\0';
    }
    
    *msglen = length;
    return buf;
}

bool parse_command(struct server *srv, char *msg_in, int msglen,
                   struct comm_item *ci) {
    char *msg = malloc(msglen*sizeof(char)+1);
    strncpy(msg, msg_in, msglen);

    /* parse command */
    char *saveptr;
    char *comm_str = strtok_r(msg, COMM_DELIM, &saveptr);
    char *arg_str = strtok_r(NULL, ARG_DELIM, &saveptr);

    struct srv_command *command = NULL;
    printf("commc: %d\n", srv->commc);
    for (int i = 0; i < srv->commc; i++) {
        struct srv_command *c = srv->comms+i;
        if (strcmp(c->name, comm_str)) {
            command = c;
            break;
        }
    }

    if (!command) goto fail;

    /* parse arguments */
    int bufsize = ARG_BUF_SIZE;
    char **args = malloc(bufsize*sizeof(char*));
    args[0] = arg_str;
    int argc = 0;
    while (args[argc]) {
        if (argc == bufsize) {
            bufsize *= 2;
            args = realloc(args, bufsize*sizeof(char*));
        }
        args[++argc] = strtok_r(NULL, ARG_DELIM, &saveptr);
    }
    args = realloc(args, argc*sizeof(char*));

    printf("command: %s\n", comm_str);
    for (int i = 0; i < argc; i++) {
        printf("arg%d: %s\n", i, args[i]);
    }

    return true;
fail:
    free(msg);
    return false;
}

void *srv_thread(void *server) {
    struct server *srv = (struct server*)server;
    bool quit = false;
    int conn_fd = -1;

    fd_set rfds;
    
    while (!quit) {
        /* sleep until connection or receive request */
        FD_ZERO(&rfds);
        FD_SET(srv->listen_fd, &rfds);
        if (conn_fd >= 0) FD_SET(conn_fd, &rfds);
        int nfds = MAX(srv->listen_fd, conn_fd)+1;
        int s = select(nfds, &rfds, NULL, NULL, NULL);

        /* accept new connection if requested */
        int conn_fd_new = accept(srv->listen_fd, NULL, NULL);
        if (conn_fd_new >= 0) {
            close(conn_fd);
            conn_fd = conn_fd_new;
        }

        /* process command if received */
        int msglen;
        char *msg = receive(conn_fd, &msglen);
        if (msglen > 1) {
            char **args;
            int argc;
            struct comm_item *ci;
            bool valid = parse_command(srv, msg, msglen, ci);
            //srv_command command = ci->command;

            /* TODO create response */
            //char *response;
            send(conn_fd, "hej", 3, 0);
            
            printf("msg: %s\n", msg);

            free(msg);
        } else {
            /* ensure remote still available */
            int sent = send(conn_fd, "hb", 2, MSG_NOSIGNAL);
            if (sent < 0) {
                close(conn_fd);
                conn_fd = -1;
            }
        }
    }

    close(conn_fd);
    pthread_exit(NULL);
}

/* external api functions */
struct server *srv_create(const char *addr, int port_start, int port_end,
                          struct srv_command *commands, int commc) {
    int succ;

    struct server *srv = calloc(1, sizeof(struct server));
    srv->comms = commands;
    srv->commc = commc;
    srv->shared.terminate = false;

    srv->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (srv->listen_fd == -1) {
        perror("failed to create unbound socket");
        goto fail;
    }

    /* ensure no blocking when accepting connections */
    fcntl(srv->listen_fd, F_SETFL, O_NONBLOCK); 

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    inet_pton(AF_INET, addr, &address.sin_addr);

    succ = 0;
    for (int port = port_start; port <= port_end; port++) {
        address.sin_port = htons(port);
        succ = bind(srv->listen_fd, (struct sockaddr*)&address, sizeof(address));
        if (succ >= 0) {
            printf("server: bound port %d\n", port);
            break;   
        }
    }

    if (succ == -1) {
        perror("server: failed to bind socket");
        goto fail;
    }

    succ = listen(srv->listen_fd, 10);
    if (succ == -1) {
        perror("failed to listen to socket");
        goto fail;
    }

    pthread_create(&srv->thread, NULL, srv_thread, (void*)(srv));

    return srv;
fail:
    srv_destroy(srv);
    return NULL;
}

void srv_destroy(struct server *srv) {
    if (srv->listen_fd > 0) close(srv->listen_fd);
    free(srv);
}

void srv_execute_commands(struct server *srv) {
            
}
