#include <stdbool.h>

typedef struct server srv_t;

struct srv_command {
    char *name;
    bool has_response;
    int min_args;
    union {
        char *(*action_response)(int argc, char **args);
        void (*action)(int argc, char **args);
    }; 
};

srv_t *srv_create(const char *addr, int port_start, int port_end,
                  struct srv_command *commands, int commc);
void srv_destroy(srv_t *server);
void srv_execute_commands(srv_t *server);
