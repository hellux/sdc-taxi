#define F_SPI 1000000

/* flags for bus command */
#define BF_WRITE 1  /* master will write */
#define BF_MOD   2  /* set values directly, or set error */
#define BF_MAN   4  /* set values directly vs error */
#define BF_REG   8  /* set regulator constants */
#define BF_VEL   16 /* set vel consts vs rot consts */

/* bus cmd bytes */
#define BCB_MAN      (BF_WRITE|BF_MOD|BF_MAN)
#define BCB_ERR      (BF_WRITE|BF_MOD)
#define BCB_REG_VEL  (BF_WRITE|              BF_REG|BF_VEL)
#define BCB_REG_ROT  (BF_WRITE              |BF_REG)
#define BCB_SENSORS  32
#define BCB_RST      64

/* data types on bus */

typedef float   ctrl_val_t;
typedef float   sens_dist_t;
typedef float   sens_odom_t;

struct ctrl_frame_data {
    ctrl_val_t value1;
    ctrl_val_t value2;
};

struct ctrl_frame_err {
    ctrl_val_t vel;
    ctrl_val_t rot;
};

struct ctrl_frame_man {
    ctrl_val_t vel;
    ctrl_val_t rot;
};

struct ctrl_frame_reg {
    ctrl_val_t kp;
    ctrl_val_t kd;
};

struct sens_frame_data {
    sens_dist_t dist_front;
    sens_dist_t dist_right;
    sens_odom_t distance;
};
