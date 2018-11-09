typedef struct server srv_t;

srv_t *srv_create(void);
void srv_destroy(srv_t *server);
void srv_listen(srv_t *server);
