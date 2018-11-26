#ifndef server_h
#define server_h

#include <stdbool.h>

typedef struct server srv_t;

/* container for arguments sent to action function */
struct srv_cmd_args {
    int argc;
    char **args;
    char *resp;
    void *data1, *data2;
};

/* helper for creating responses */
/* str must be valid string with *buf_size bytes allocated */
char *str_append(char *str, int *buf_size, const char *fmt, ...);

/*
 * data1/data2:
 *      -will be sent to action/response
 *      -used to forward structs with data that will be returned or modified
 * action:
 *      -called when cmd received
 *      -called from server thread, must be synchronized
 *      -returned string will be freed by server thread
 *      -shall write ptr to response (if any) to a->resp
 *      -shall return true if command was valid
 */
struct srv_cmd {
    char *name;
    int min_args;
    void *data1, *data2;
    bool (*action)(struct srv_cmd_args *a);
};

srv_t *srv_create(const char *addr, int port_start, int port_end,
                  struct srv_cmd *cmds, int cmdc);
void srv_destroy(srv_t *server);

#endif
