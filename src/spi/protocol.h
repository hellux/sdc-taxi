#ifndef protocol_h
#define protocol_h

#include <stdint.h>
#include <stdbool.h>

/* message structure:
 *  |<cs>|<dt1>|..|<dtn>|
 *  a cmd sum byte containing command and checksum followed by variable amount
 *  of data bytes */

#define F_SPI 500000

/* IDs for slaves */
#define SLAVE_SENS 0
#define SLAVE_CTRL 1

/* SYN/ACK magic values */
#define CTRL_ACK 0xc7
#define SENS_ACK 0x53

/* data types on bus */

typedef float ctrl_val_t;
typedef float sens_dist_t;
typedef float sens_odom_t;

/* cmd sum type */

typedef uint8_t cs_t;

uint8_t cs_cmd(cs_t cs);
cs_t cs_create(uint8_t cmd, void *data, int len);
bool cs_check(uint8_t cmd, void *data, int len);

struct bus_cmd {
    uint8_t cmd;
    bool write;
    int slave;
    int len;
};

#define BB_INVALID 0

#define BF_WRITE    8
#define BF_VEL_ROT  4
#define BF_MOD_REG  2
#define BF_ERR_VAL  1
#define BF_KP_KD    1 /* should share with above */

/* ctrl commands */
#define BBC_RST     0x01
#define BBC_SYN     0x02
#define BBC_ROT_KD  (BF_WRITE)          
#define BBC_ROT_KP  (BF_WRITE|                      BF_KP_KD)
#define BBC_ROT_VAL (BF_WRITE|           BF_MOD_REG)
#define BBC_ROT_ERR (BF_WRITE|           BF_MOD_REG|BF_ERR_VAL)
#define BBC_VEL_KD  (BF_WRITE|BF_VEL_ROT)
#define BBC_VEL_KP  (BF_WRITE|BF_VEL_ROT|           BF_KP_KD)
#define BBC_VEL_VAL (BF_WRITE|BF_VEL_ROT|BF_MOD_REG)
#define BBC_VEL_ERR (BF_WRITE|BF_VEL_ROT|BF_MOD_REG|BF_ERR_VAL)

static struct bus_cmd BCCS[16] = {
/*  cmd           write, slave id,   data length */
    {BB_INVALID,  false, 0,          0},
    {BBC_RST,     false, SLAVE_CTRL, 0},
    {BBC_SYN,     false, SLAVE_CTRL, sizeof(ctrl_val_t)},
    {0},

    {0}, {0}, {0}, {0},

    {BBC_ROT_KD,  true,  SLAVE_CTRL, sizeof(ctrl_val_t)},
    {BBC_ROT_KP,  true,  SLAVE_CTRL, sizeof(ctrl_val_t)},
    {BBC_ROT_VAL, true,  SLAVE_CTRL, sizeof(ctrl_val_t)},
    {BBC_ROT_ERR, true,  SLAVE_CTRL, sizeof(ctrl_val_t)},

    {BBC_VEL_KD,  true,  SLAVE_CTRL, sizeof(ctrl_val_t)},
    {BBC_VEL_KP,  true,  SLAVE_CTRL, sizeof(ctrl_val_t)},
    {BBC_VEL_VAL, true,  SLAVE_CTRL, sizeof(ctrl_val_t)},
    {BBC_VEL_ERR, true,  SLAVE_CTRL, sizeof(ctrl_val_t)},
};

struct sens_data {
    sens_dist_t dist_front;
    sens_dist_t dist_right;
    sens_odom_t distance;
};

/* sens commands */
#define BBS_RST 0x01
#define BBS_SYN 0x02
#define BBS_GET 0x03

/* bus commands for sens */
static struct bus_cmd BCSS[16] = {
/*  cmd          write, slave id,   data length */
    {BB_INVALID, false, 0,          0},
    {BBS_RST,    false, SLAVE_SENS, 0},
    {BBS_SYN,    false, SLAVE_SENS, 1},
    {BBS_GET,    false, SLAVE_SENS, sizeof(struct sens_data)},

    {0}, {0}, {0}, {0},
    {0}, {0}, {0}, {0},
    {0}, {0}, {0}, {0},
};

#endif
