#include <stdio.h>

#include "../comm/ip/img_proc.h"

int main(int argc, char* args[]) {
    ip_t *ip = ip_init();
    struct ip_res result;
    while (ip) {
        ip_process(ip, &result, NULL);
    }
    ip_destroy(ip);

    return 0;
}
