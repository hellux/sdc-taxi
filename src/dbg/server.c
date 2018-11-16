#include "../comm/server.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char *hej(int argc, char **args, void *data) {
    char *str = "hej hej";
    char *response = malloc(sizeof(str));
    strcpy(response, str);
    return response;
}

void action_test(int argc, char **args, void *data) {
    printf("hej hej\n");
    char *str = (char*)data;
    printf("%s\n", str);
}

int main(int argc, char* args[]) {
    struct srv_command commands[] = {
        {"hej", 0, NULL, true, {response:*hej}},
        {"make", 0, "hej hej", false, {action:*action_test}},
    };
    int commc = 2;
    srv_t *srv = srv_create("127.0.0.1", 9000, 10000, commands, commc);

    char comm[100];
    while (1) {
       scanf("%s", comm);
       if (comm[0] == 'q')
           break;
       srv_execute_commands(srv);
    }

    srv_destroy(srv);

    return 0;
}
