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
#define VEL_MIN -0.2
#define VEL_MAX 0.2
#define ROT_MIN -0.95
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
    
    volatile struct pd_values *pdv = (command & BF_VEL_ROT) ? &vel : &rot;

    if (command & BF_WRITE) {
        ctrl_val_t value_retrieved;
        spi_tranceive((uint8_t*)&value_retrieved, sizeof(value_retrieved));

        if (command & BF_MOD_REG) {
            ctrl_val_t value_new;

            if (command & BF_ERR_VAL) {
                pdv->err_prev = pdv->err;
                pdv->err = value_retrieved;
                value_new = pd_ctrl(pdv);
            } else {
                value_new = value_retrieved;
            }

            float min = (command & BF_VEL_ROT) ? VEL_MIN : ROT_MIN;
            float max = (command & BF_VEL_ROT) ? VEL_MAX : ROT_MAX;
            value_new = MIN(MAX(value_new, min), max);
            float duty = DUTY_NEUTRAL + value_new*(DUTY_MAX-DUTY_NEUTRAL);

            if (command & BF_VEL_ROT) {
                OCR_VEL = duty*PWM_TOP; 
            } else {
                OCR_ROT = duty*PWM_TOP; 
            }
        } else {
            volatile ctrl_val_t *dst =
                (command & BF_KP_KD) ? &pdv->kp : &pdv->kd;
            *dst = value_retrieved;
        }
    } else if (command == BCBC_RST) {
        reset();
    } else if (command == BCBC_SYN) {
        uint8_t ack = CTRL_ACK;
        spi_tranceive(&ack, sizeof(ack));
    }

    sei();
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
