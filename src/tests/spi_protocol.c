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

    /* check that commands are mapped correct */
    for (int i = 0; i < sizeof(CTRL_CMDS)/sizeof(*CTRL_CMDS); i++) {
        int command = CTRL_CMDS[i];
        assert (BCCS[command].cmd == command);
    }
    for (int i = 0; i < sizeof(SENS_CMDS)/sizeof(*SENS_CMDS); i++) {
        int command = SENS_CMDS[i];
        assert (BCSS[command].cmd == command);
    }

    return 0;
}
