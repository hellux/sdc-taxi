#ifndef protocol_h

#define F_SPI 1000000

#define BF_WRITE    1
#define BF_VEL_ROT  2
#define BF_MOD_REG  4
#define BF_ERR_VAL  8
#define BF_KP_KD    8 /* should share with above */

/* ctrl commands */
#define BCBC_VEL_VAL (BF_WRITE|BF_VEL_ROT|BF_MOD_REG)
#define BCBC_VEL_ERR (BF_WRITE|BF_VEL_ROT|BF_MOD_REG|BF_ERR_VAL)
#define BCBC_VEL_KP  (BF_WRITE|BF_VEL_ROT|           BF_KP_KD)
#define BCBC_VEL_KD  (BF_WRITE|BF_VEL_ROT)
#define BCBC_ROT_VAL (BF_WRITE|           BF_MOD_REG)
#define BCBC_ROT_ERR (BF_WRITE|           BF_MOD_REG|BF_ERR_VAL)
#define BCBC_ROT_KP  (BF_WRITE|                      BF_KP_KD)
#define BCBC_ROT_KD  (BF_WRITE)          
#define BCBC_RST     16
#define BCBC_SYN     32

/* sens commands */
#define BCBS_GET 1
#define BCBS_RST 2
#define BCBS_SYN 3

/* SYN/ACK magic values */
#define CTRL_ACK 0xc7
#define SENS_ACK 0x53

/* data types on bus */

typedef float   ctrl_val_t;
typedef float   sens_dist_t;
typedef float   sens_odom_t;

struct ctrl_err {
    ctrl_val_t vel;
    ctrl_val_t rot;
};

struct ctrl_man {
    ctrl_val_t vel;
    ctrl_val_t rot;
};

struct ctrl_reg {
    ctrl_val_t kp;
    ctrl_val_t kd;
};

struct sens_data {
    sens_dist_t dist_front;
    sens_dist_t dist_right;
    sens_odom_t distance;
};

#endif
