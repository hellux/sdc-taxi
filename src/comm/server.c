#include "server.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <pthread.h>

#include <sys/select.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#define COMM_DELIM ":"
#define ARG_DELIM ","

#define RSP_SUCCESS_PRE "success:"
#define RSP_FAILURE_PRE "failure:"
#define RSP_INVALID_ARG "invalid_arg"
#define RSP_INVALID_CMD "invalid_cmd"

#define MSG_BUF_SIZE 2048   /* starting size for message length */
#define RSP_BUF_SIZE 2048   /* max size for resp length */
#define ARG_BUF_SIZE 64     /* starting size for arg count */

#define MAX(A, B) ((A) > (B) ? (A) : (B))

struct server {
    int listen_fd; /* fd for socket that is listened to */

    struct srv_cmd *cmds;
    int cmdc;

    bool terminate;
    pthread_t thread;
    pthread_mutex_t lock;
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

/* modifies msg */
bool parse_cmd(struct server *srv, char *msg, int msglen,
               struct srv_cmd **cmd_dst, int *argc_dst, char ***args_dst) {
    char *saveptr;
    char *cmd_str = strtok_r(msg, COMM_DELIM, &saveptr);
    char *arg_str = strtok_r(NULL, ARG_DELIM, &saveptr);

    struct srv_cmd *cmd = NULL;
    for (int i = 0; i < srv->cmdc; i++) {
        struct srv_cmd *c = srv->cmds+i;
        if (strcmp(c->name, cmd_str) == 0) {
            cmd = c;
            break;
        }
    }

    if (!cmd) return false;

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

    if (argc < cmd->min_args) {
        free(args);
        return false;
    }

    *cmd_dst = cmd;
    *argc_dst = argc;
    *args_dst = args;

    return true;
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
        select(nfds, &rfds, NULL, NULL, NULL);

        /* accept new connection if requested */
        int conn_fd_new = accept(srv->listen_fd, NULL, NULL);
        if (conn_fd_new >= 0) {
            close(conn_fd);
            conn_fd = conn_fd_new;
        }

        /* process cmd if received */
        int msglen;
        char *msg = receive(conn_fd, &msglen);
        if (msglen > 1) {
            struct srv_cmd *cmd;
            int argc;
            char **args;
            bool valid = parse_cmd(srv, msg, msglen, &cmd, &argc, &args);
            if (valid) {
                char *response;
                bool success = cmd->action(argc, args, &response,
                                           cmd->data1, cmd->data2);
                char* prefix = success ? RSP_SUCCESS_PRE : RSP_FAILURE_PRE;
                char* msgrsp;
                if (response) {
                    int prefix_len = strlen(prefix);
                    int response_len = strlen(response);
                    msgrsp = malloc(prefix_len+response_len+1);
                    strcpy(msgrsp, prefix);
                    strcpy(msgrsp+prefix_len, response);
                } else {
                    msgrsp = prefix;
                }
                send(conn_fd, msgrsp, strlen(msgrsp), 0);
            } else {
                char* response = RSP_FAILURE_PRE RSP_INVALID_CMD;
                send(conn_fd, response, strlen(response), 0);
            }
            free(msg);
        } else {
            /* ensure remote still available, otherwise close socket */
            char *msg = "heartbeat";
            int sent = send(conn_fd, msg, strlen(msg), MSG_NOSIGNAL);
            if (sent < 0) {
                close(conn_fd);
                conn_fd = -1;
            }
        }
        pthread_mutex_lock(&srv->lock);
        quit = srv->terminate;
        pthread_mutex_unlock(&srv->lock);
    }

    close(conn_fd);
    pthread_exit(NULL);
}

/* external api functions */
struct server *srv_create(const char *addr, int port_start, int port_end,
                          struct srv_cmd *cmds, int cmdc) {
    int succ;

    struct server *srv = calloc(1, sizeof(struct server));
    srv->cmds = cmds;
    srv->cmdc = cmdc;
    srv->terminate = false;
    pthread_mutex_init(&srv->lock, 0);

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
    pthread_mutex_lock(&srv->lock);
    srv->terminate = true;
    pthread_mutex_unlock(&srv->lock);

    pthread_join(srv->thread, NULL);

    if (srv->listen_fd > 0) close(srv->listen_fd);
    pthread_mutex_destroy(&srv->lock);

    free(srv);
}
