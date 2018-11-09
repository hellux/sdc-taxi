#include "../comm/server.h"

int main(int argc, char* args[]) {
    srv_t *srv = srv_create("127.0.0.1", 5000);

    while (1) {
        srv_listen(srv);
    }

    srv_destroy(srv);

    return 0;
}
