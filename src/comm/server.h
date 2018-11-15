#include <stdbool.h>

typedef struct server srv_t;

struct srv_command {
    char *name;
    bool has_response;
    union {
        char **action_response;
        void *action;
    }; 
};

srv_t *srv_create(const char *addr, int port);
void srv_destroy(srv_t *server);
