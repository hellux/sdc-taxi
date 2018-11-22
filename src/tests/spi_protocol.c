#include <assert.h>

#include "spi/protocol.h"

#define BCS_MATCH(array, command) assert (array[command].cmd == command)

int main(void) {
    /* ctrl bcs are mapped correctly */
    BCS_MATCH(BCCS, BB_INVALID);
    BCS_MATCH(BCCS, BBC_RST);
    BCS_MATCH(BCCS, BBC_SYN);
    BCS_MATCH(BCCS, BBC_ROT_KD);
    BCS_MATCH(BCCS, BBC_ROT_KP);
    BCS_MATCH(BCCS, BBC_ROT_VAL);
    BCS_MATCH(BCCS, BBC_ROT_ERR);
    BCS_MATCH(BCCS, BBC_VEL_KD);
    BCS_MATCH(BCCS, BBC_VEL_KP);
    BCS_MATCH(BCCS, BBC_VEL_VAL);
    BCS_MATCH(BCCS, BBC_VEL_ERR);

    /* sens bcs are mapped correctly */
    BCS_MATCH(BCSS, BB_INVALID);
    BCS_MATCH(BCSS, BBS_RST);
    BCS_MATCH(BCSS, BBS_SYN);
    BCS_MATCH(BCSS, BBS_GET);

    return 0;
}
