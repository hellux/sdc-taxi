#define F_SPI 4000000

/* flags for bus command;
 *  examples:
 *      write speed =   0000 0011 = 3
 *      write turn kp = 0000 0101 = 5 */
#define BF_WRITE 1
#define BF_VEL 2
#define BF_REG   4
#define BF_KD    8

/* bus cmd bytes */
#define BCB_VEL      (BF_WRITE|BF_VEL)
#define BCB_VEL_KD   (BF_WRITE|BF_VEL|BF_REG|BF_KD)
#define BCB_VEL_KP   (BF_WRITE|BF_VEL|BF_REG)
#define BCB_ROT      (BF_WRITE)
#define BCB_ROT_KD   (BF_WRITE|       BF_REG|BF_KD)
#define BCB_ROT_KP   (BF_WRITE|       BF_REG)
#define BCB_GET_SENS 16
#define BCB_RESET    32

typedef float   sens_dist_t;
typedef float   sens_rot_t;

typedef float   ctrl_const_t;
typedef float   ctrl_err_t;

/* format of data sent from sensor via bus */
struct sens_data_frame {
    sens_dist_t dist_front;
    sens_dist_t dist_side;
    sens_rot_t dist_wheel;
};
