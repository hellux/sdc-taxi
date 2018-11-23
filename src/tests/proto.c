#include <assert.h>

#include "spi/protocol.h"

int main(void) {
    /* must contain all possible commands */
    const int CTRL_CMDS[] = {
        BB_INVALID,
        BBC_RST,
        BBC_SYN,
        BBC_ROT_KD,
        BBC_ROT_KP,
        BBC_ROT_VAL,
        BBC_ROT_ERR,
        BBC_VEL_KD,
        BBC_VEL_KP,
        BBC_VEL_VAL,
        BBC_VEL_ERR,
    };
    const int SENS_CMDS[] = {
        BB_INVALID,
        BBS_RST,
        BBS_SYN,
        BBS_GET,
    };
    int CTRL_CMDC = sizeof(CTRL_CMDS)/sizeof(*CTRL_CMDS);
    int SENS_CMDC = sizeof(SENS_CMDS)/sizeof(*SENS_CMDS);

    assert (CTRL_CMDC > 0);
    assert (SENS_CMDC > 0);

    /* check that commands are mapped correct */
    for (int i = 0; i < CTRL_CMDC; i++) {
        int command = CTRL_CMDS[i];
        assert (BCCS[command].cmd == command);
    }
    for (int i = 0; i < SENS_CMDC; i++) {
        int command = SENS_CMDS[i];
        assert (BCSS[command].cmd == command);
    }

    double data1 = 34324.98;
    double data1_copy = data1;
    uint8_t cmd1 = BBC_ROT_VAL;
    cs_t cs1 = cs_create(cmd1, (void*)&data1, sizeof(data1));
    assert (cs_check(cs1, (void*)&data1, sizeof(data1)));
    assert (cs_check(cs1, (void*)&data1_copy, sizeof(data1_copy)));
    assert (cs_cmd(cs1) == cmd1);

    uint8_t data2[100];
    uint8_t cmd2;
    cs_t cs2 = cs_create(cmd2, (void*)&data2, sizeof(data2));
    assert (cs_check(cs2, (void*)&data2, sizeof(data2)));
    assert (cs_cmd(cs2) == cmd2);


    return 0;
}
