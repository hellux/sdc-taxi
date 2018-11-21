#include "bus.h"
#include "lcd.h"
#include "jtag.h"

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#include "../spi/protocol.h"

#ifdef SIM
#include "../dbg/sim/ctrl_header.h"
#endif

#define MIN(A, B) ((A) < (B) ? (A) : (B))
#define MAX(A, B) ((A) > (B) ? (A) : (B))

#define DUTY_MAX 0.10
#define DUTY_NEUTRAL 0.075
#define DUTY_MIN 0.050
#define VEL_MAX 0.2
#define ROT_MAX 0.95

#define PWM_T 0.020
#define PWM_PSC 16
#define PWM_TOP F_CPU*PWM_T/PWM_PSC

#define OCR_VEL OCR1B
#define OCR_ROT OCR1A

/* regulator constants at start */
#define VEL_KP_DEF 0.1
#define VEL_KD_DEF 0.0

#define ROT_KP_DEF 0.1
#define ROT_KD_DEF 0.0

struct pd_values {
    ctrl_val_t kp;
    ctrl_val_t kd;
    ctrl_val_t err;
    ctrl_val_t err_prev;
};

const struct pd_values PD_EMPTY = {0};

volatile struct pd_values vel;
volatile struct pd_values rot;

void reset() {
    vel = PD_EMPTY;
    vel.kp = VEL_KP_DEF;
    vel.kd = VEL_KD_DEF;
    rot = PD_EMPTY;
    rot.kp = ROT_KP_DEF;
    rot.kd = ROT_KD_DEF;
    OCR_VEL = DUTY_NEUTRAL*PWM_TOP;
    OCR_ROT = DUTY_NEUTRAL*PWM_TOP;
}

float pd_ctrl(volatile struct pd_values *v){
    float proportion =  v->err                * v->kp;
    float derivative = (v->err - v->err_prev) * v->kd;

    return proportion + derivative;
}

ISR(SPI_STC_vect){
    cli();
    uint8_t command;
    spi_tranceive(&command, sizeof(command));
    
    /* retrieve data if write command */
    struct ctrl_frame_data frame;
    if (command & BF_WRITE) {
        spi_tranceive((uint8_t*)&frame, sizeof(frame));
    }

    /* set new values */
    if (command == BCB_ERR) {
        struct ctrl_frame_err *new = (struct ctrl_frame_err*)&frame;
        vel.err_prev = vel.err;
        vel.err = new->vel;
        rot.err_prev = rot.err;
        rot.err = new->rot;
    } else if (command & BF_REG) {
        struct ctrl_frame_reg *new = (struct ctrl_frame_reg*)&frame;
        volatile struct pd_values *reg = (command & BF_VEL) ? &vel : &rot;
        reg->kp = new->kp;
        reg->kd = new->kd;
    } else if (command == BCB_RST) {
        reset();
    }

    /* update motor values */
    if (command & BF_MOD) {
        struct ctrl_frame_man new;

        if (command & BF_MAN) {
            new = *((struct ctrl_frame_man*)&frame);
        } else {
            new.vel = pd_ctrl(&vel); /* TODO use pd_ctrl when not testing */
            new.rot = pd_ctrl(&rot);
        }

        new.vel = MIN(MAX(new.vel, -VEL_MAX), VEL_MAX);
        new.rot = MIN(MAX(new.rot, -ROT_MAX), ROT_MAX);

        float duty_vel = DUTY_NEUTRAL + new.vel*(DUTY_MAX-DUTY_NEUTRAL);
        float duty_rot = DUTY_NEUTRAL + new.rot*(DUTY_MAX-DUTY_NEUTRAL);

        OCR_VEL = duty_vel*PWM_TOP; 
        OCR_ROT = duty_rot*PWM_TOP; 
        
        sei();
    }
}

void pwm_init(){
    /* Initialize to phase and frequency correct PWM */
    TCCR1A |= (1<<COM1A1)|(1<<COM1B1);
    TCCR1B |= (1<<WGM13)|(1<<CS11);

    ICR1 = PWM_TOP;

    /* set outputs */
    DDRD |= (1<<PD4)|(1<<PD5);
}

int main() {
    pwm_init();
    spi_init_slave();
    init_jtagport();
    reset();

    sei();
    while (1) {
        /*
        _delay_ms(3000);
        */
    }
}
