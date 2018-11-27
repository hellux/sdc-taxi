#include <stdio.h>

#include "../comm/ip/img_proc.h"

int main(int argc, char* args[]) {
    printf("hej från C\n");

    ip_t *ip = ip_init();
    struct ip_res result;
    ip_process(ip, &result);

    return 0;
}
