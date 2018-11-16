#include <stdbool.h>

typedef struct server srv_t;

/*
 * data:
 *      -will be sent to action/response
 *      -used to forward structs with data that will be returned or modified
 * response:
 *      -called if has_response
 *      -called when cmd received
 *      -called from server thread, must be synchronized
 *      -returned string will be freed by server thread
 * action:
 *      -called if has_response is false
 *      -called from main thread via srv_execute_cmds()
 */
struct srv_cmd {
    char *name;
    int min_args;
    void *data;
    bool has_response;
    union {
        void (*action)(int argc, char **args, void *data);
        char *(*response)(int argc, char **args, void *data);
    } func; 
};

srv_t *srv_create(const char *addr, int port_start, int port_end,
                  struct srv_cmd *cmds, int cmdc);
void srv_destroy(srv_t *server);
void srv_execute_cmds(srv_t *server);
