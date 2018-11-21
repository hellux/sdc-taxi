#define F_SPI 2000000

/* flags for bus command */
#define BF_WRITE 1 /* master will write */
#define BF_REG   4 /* regulator vs error constant */
#define BF_VEL   2 /* velocity vs rotation */

/* bus cmd bytes */
#define BCB_ERR      (BF_WRITE)
#define BCB_REG_VEL  (BF_WRITE|BF_REG|BF_VEL)
#define BCB_REG_ROT  (BF_WRITE|BF_REG)
#define BCB_SENSORS  16
#define BCB_RST      32

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

struct ctrl_frame_reg {
    ctrl_val_t kp;
    ctrl_val_t kd;
};

struct sens_frame_data {
    sens_dist_t dist_front;
    sens_dist_t dist_right;
    sens_odom_t distance;
};
