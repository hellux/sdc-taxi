#include "server.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <pthread.h>

#include <sys/select.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#define CMD_DELIM ":"
#define ARG_DELIM ","

#define RSP_SUCCESS_PRE "success:"
#define RSP_FAILURE_PRE "failure:"
#define RSP_INVALID_ARG "invalid_arg"
#define RSP_INVALID_CMD "invalid_cmd"

#define MSG_BUF_SIZE 2048   /* starting size for message length */
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

/* built in commands */

bool cmd_check(struct srv_cmd_args *a) {
    const int BUF_START = 30;
    int bufs = BUF_START;
    char *rsp = malloc(bufs);
    char *str_pos = rsp;
    str_pos += sprintf(str_pos, "argc: %d, args: ", a->argc);

    if (a->argc > 0) {
        for (int i = 0; i < a->argc; i++) {
            int max_size = bufs-(str_pos-rsp);
            int len = snprintf(str_pos, max_size, "\"%s\", ", a->args[i]);
            if (len < max_size) {
                str_pos += len;
            } else {
                int total_length = str_pos-rsp;
                bufs *= 2;
                rsp = realloc(rsp, bufs);
                str_pos = rsp+total_length;
                i--;
            }
        }
        /* erase last comma and space */
        str_pos -= 2;
        str_pos += sprintf(str_pos, ".");
    }

    *a->resp = rsp;
    return true;
}

bool cmd_help(struct srv_cmd_args *a) {
    char *response = malloc(1024);
    struct srv_cmd *cmds = *(struct srv_cmd**)a->data1;
    int cmdc = *(int*)a->data2;
    char *str_pos = response;
    str_pos += sprintf(str_pos, "available commands: ");
    for (int i = 0; i < cmdc; i++) {
        str_pos += sprintf(str_pos, "%s, ", cmds[i].name);
    }
    /* erase last comma and space */
    str_pos -= 2;
    str_pos += sprintf(str_pos, ".");

    *a->resp = response;
    return true;
}

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

/* 
 * retrieve command, argc and arguments from string msg
 * store result in cmd_dst, argc_dst and args_dst
 *
 * return:
 *      true if valid command with more than min arguments
 *      false if invalid command
 *
 * side effects:
 *      modify msg: add null chars between arguments */
bool parse_cmd(struct server *srv, char *msg, int msglen,
               struct srv_cmd **cmd_dst, int *argc_dst, char ***args_dst) {
    char *saveptr;
    char *cmd_str = strtok_r(msg, CMD_DELIM, &saveptr);
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

/* parse and execute command (if valid) in msg
 * retrieve and return response from command */
char* execute_cmd(struct server *srv, char* msg, int msglen) {
    struct srv_cmd *cmd;
    int argc;
    char **args;
    bool valid = parse_cmd(srv, msg, msglen, &cmd, &argc, &args);
    char *msg_rsp;

    if (valid) {
        char *response;
        struct srv_cmd_args a = {
            argc, args, &response, cmd->data1, cmd->data2
        };
        bool success = cmd->action(&a);
        char* prefix = success ? RSP_SUCCESS_PRE : RSP_FAILURE_PRE;
        if (response) {
            int prefix_len = strlen(prefix);
            int response_len = strlen(response);
            msg_rsp = malloc(prefix_len+response_len+1);
            strcpy(msg_rsp, prefix);
            strcpy(msg_rsp+prefix_len, response);
        } else {
            msg_rsp = prefix;
        }
    } else {
        msg_rsp = RSP_FAILURE_PRE RSP_INVALID_CMD;
    }

    return msg_rsp;
}

void *srv_thread(void *server) {
    struct server *srv = (struct server*)server;
    bool quit = false;
    int conn_fd = -1;

    fd_set rfds;
    
    while (!quit) {
        /* sleep until request for connection or receive */
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

        /* handle msg if received */
        int msglen;
        char *msg = receive(conn_fd, &msglen);
        if (msglen > 0) {
            char *msg_rsp = execute_cmd(srv, msg, msglen);
            send(conn_fd, msg_rsp, strlen(msg_rsp), 0);
            free(msg);
        } else {
            /* ensure remote still available, otherwise close socket */
            char *msg_rsp = "heartbeat";
            int sent = send(conn_fd, msg_rsp, strlen(msg_rsp), MSG_NOSIGNAL);
            if (sent < 0) {
                close(conn_fd);
                conn_fd = -1;
            }
        }

        /* check if server should be destroyed */
        pthread_mutex_lock(&srv->lock);
        quit = srv->terminate;
        pthread_mutex_unlock(&srv->lock);
    }

    close(conn_fd);
    pthread_exit(NULL);
}

/* external api functions */

struct server *srv_create(const char *addr, int port_start, int port_end,
                          struct srv_cmd *cmds_in, int cmdc_in) {
    struct server *srv = calloc(1, sizeof(struct server));
    srv->terminate = false;
    pthread_mutex_init(&srv->lock, 0);

    /* add commands, built in and from caller */
    struct srv_cmd cmds_std[] = {
        {"help",            0, &srv->cmds, &srv->cmdc, *cmd_help},
        {"check",           0, NULL, NULL, *cmd_check},
    };
    int cmdc_std = sizeof(cmds_std)/sizeof(*cmds_std);
    int cmdc = cmdc_std+cmdc_in;
    struct srv_cmd *cmds = malloc(cmdc*sizeof(*cmds));
    memcpy(cmds, cmds_std, sizeof(cmds_std));
    memcpy(cmds+cmdc_std, cmds_in, cmdc_in*sizeof(*cmds_in));
    srv->cmds = cmds;
    srv->cmdc = cmdc;

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

    int succ = 0;
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
    if (srv) {
        pthread_mutex_lock(&srv->lock);
        srv->terminate = true;
        pthread_mutex_unlock(&srv->lock);

        pthread_join(srv->thread, NULL);

        if (srv->listen_fd > 0) close(srv->listen_fd);
        pthread_mutex_destroy(&srv->lock);

        free(srv->cmds);
        free(srv);
    }
}
