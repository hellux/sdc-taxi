#include "protocol.h"

struct cmd_sum {
    uint8_t cmd:4;
    uint8_t sum:4;
};

uint8_t cs_cmd(cs_t cs) {
    return ((struct cmd_sum*)&cs)->cmd;
}

uint8_t cs_sum(cs_t cs) {
    return ((struct cmd_sum*)&cs)->sum;
}

uint8_t calc_sum(uint8_t cmd, void *data, int len) {
    /* TODO use better algorithm */
    uint8_t sum = 0;
    sum ^= cmd;
    for (int i = 0; i < len; i++) {
        sum ^= *((uint8_t*)data);
        sum ^= *((uint8_t*)data) >> 4;
    }

    return sum & 0x0f;
}

cs_t cs_create(uint8_t cmd, void *data, int len) {
    uint8_t sum = calc_sum(cmd, data, len);
    struct cmd_sum css;
    css.sum = sum;
    css.cmd = cmd;
    return *(cs_t*)&css;
}

bool cs_check(cs_t cs, void *data, int len) {
    return cs_sum(cs) == calc_sum(cs_cmd(cs), data, len);
}
