#include "../comm/server.h"
#include <unistd.h>

int main(int argc, char* args[]) {
    srv_t *srv = srv_create("127.0.0.1", 9849);

    while (1) {
        sleep(1);
    }

    srv_destroy(srv);

    return 0;
}
