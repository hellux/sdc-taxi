#define F_SPI 4000000

/* flags for bus command;
 *  examples:
 *      write speed =   0000 0011 = 3
 *      write turn kp = 0000 0101 = 5 */
#define BF_WRITE 1
#define BF_SPEED 2
#define BF_REG   4
#define BF_KD    8

/* bus cmd bytes */
#define BCB_SPEED    (BF_WRITE|BF_SPEED)
#define BCB_SPEED_KD (BF_WRITE|BF_SPEED|BF_REG|BF_KD)
#define BCB_SPEED_KP (BF_WRITE|BF_SPEED|BF_REG)
#define BCB_TURN     (BF_WRITE)
#define BCB_TURN_KD  (BF_WRITE|         BF_REG|BF_KD)
#define BCB_TURN_KP  (BF_WRITE|         BF_REG)
#define BCB_GET_SENS 16
#define BCB_RESET    32

typedef float   sens_dist_t;
typedef uint8_t sens_rot_t;

typedef float   ctrl_const_t;
typedef float   ctrl_err_t;

/* format of data sent from sensor via bus */
struct sens_data_frame {
    sens_dist_t dist_front;
    sens_dist_t dist_right;
    sens_rot_t rotations;
};
