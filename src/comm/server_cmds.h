#include "protocol.h"
#include "server.h"

bool sc_get_sens(struct srv_cmd_args *a);
bool sc_get_mission(struct srv_cmd_args *a);
bool sc_set_mission(struct srv_cmd_args *a);
bool sc_set_bool(struct srv_cmd_args *a);
bool sc_set_float(struct srv_cmd_args *a);
bool sc_bus_send_float(struct srv_cmd_args *a);
