#include <stdbool.h>

typedef struct server srv_t;

/*
 * data1/data2:
 *      -will be sent to action/response
 *      -used to forward structs with data that will be returned or modified
 *      -generally first for modifiable data and second for mutex
 * action:
 *      -called if has_response
 *      -called when cmd received
 *      -called from server thread, must be synchronized
 *      -returned string will be freed by server thread
 */
struct srv_cmd {
    char *name;
    int min_args;
    void *data1, *data2;
    bool (*action)(int argc, char **args, char **resp,
                   void *data1, void *data2);
};

srv_t *srv_create(const char *addr, int port_start, int port_end,
                  struct srv_cmd *cmds, int cmdc);
void srv_destroy(srv_t *server);
