typedef struct server srv_t;

srv_t *srv_create(const char *addr, int port);
void srv_destroy(srv_t *server);
void srv_listen(srv_t *server);
