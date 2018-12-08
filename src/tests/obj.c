#include <assert.h>
#include <string.h>
#include <stdio.h>

#include "comm/objective.h"

int main(void) {
    obj_t *obj = obj_create();

    assert(!obj_active(obj));
    assert(obj_remaining(obj) == 0);

    char *cmds1[] = {"hej", "hejsan"};
    assert(!obj_set_mission(obj, sizeof(cmds1)/sizeof(*cmds1), cmds1));
    assert(obj_remaining(obj) == 0);

    char *cmds2[] = {"entr", "cont", "exit"};
    assert(obj_set_mission(obj, sizeof(cmds2)/sizeof(*cmds2), cmds2));
    assert(obj_remaining(obj) == sizeof(cmds2)/sizeof(*cmds2));

    obj_set_state(obj, true);
    assert(obj_active(obj));

    assert(obj_set_mission(obj, 0, NULL));
    assert(obj_remaining(obj) == 0);

    obj_destroy(obj);

    return 0;
}
