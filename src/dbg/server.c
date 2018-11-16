#include "../comm/server.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool hej(int argc, char **args, char *rsp_dst, void *d1, void *d2) {
    rsp_dst = "hej hej";
    return true;
}

void action_test(int argc, char **args, void *data) {
    printf("hej hej\n");
    char *str = (char*)data;
    printf("%s\n", str);
}

int main(int argc, char* args[]) {
    struct srv_cmd cmds[] = {
        {"hej", 0, NULL, NULL, *hej},
    };
    int cmdc = 1;
    printf("size: %ld, %ld\n", sizeof(cmds), sizeof(*cmds));
    srv_t *srv = srv_create("127.0.0.1", 9000, 10000, cmds, cmdc);

    char cmd[100];
    while (1) {
       scanf("%s", cmd);
       if (cmd[0] == 'q')
           break;
    }

    srv_destroy(srv);

    return 0;
}
