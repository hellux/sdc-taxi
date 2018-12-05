#include "bus.h"

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#include "protocol.h"

#define MIN(A, B) ((A) < (B) ? (A) : (B))
#define MAX(A, B) ((A) > (B) ? (A) : (B))

/* PWM */
#define DUTY_MAX 0.10
#define DUTY_NEUTRAL 0.075
#define DUTY_MIN 0.050
#define VEL_MIN 0.06
#define VEL_MAX 0.11
#define VEL_MIN_NEG 0.06
#define VEL_MAX_NEG 0.13
#define ROT_MIN 0
#define ROT_MAX 0.95

#define PWM_T 0.020
#define PWM_PSC 16
#define PWM_TOP F_CPU*PWM_T/PWM_PSC

#define OCR_VEL OCR1B
#define OCR_ROT OCR1A

#define VEL_KP_DEF 1
#define VEL_KD_DEF 0.0

#define ROT_KP_DEF 2.3
#define ROT_KD_DEF 20

/* killswitch */
#define KS_OCR OCR3A
#define KS_TCNT TCNT3

#define KS_T 0.2
#define KS_PSC 64
#define KS_TOP F_CPU*KS_T/KS_PSC

struct pd_values {
    ctrl_val_t kp;
    ctrl_val_t kd;
    ctrl_val_t err;
    ctrl_val_t err_prev;
};

const struct pd_values PD_EMPTY = {0};

volatile struct pd_values vel;
volatile struct pd_values rot;

void reset(void) {
    cli();
    vel = PD_EMPTY;
    vel.kp = VEL_KP_DEF;
    vel.kd = VEL_KD_DEF;
    rot = PD_EMPTY;
    rot.kp = ROT_KP_DEF;
    rot.kd = ROT_KD_DEF;
    OCR_VEL = DUTY_NEUTRAL*PWM_TOP;
    OCR_ROT = DUTY_NEUTRAL*PWM_TOP;
    sei();
}

float pd_ctrl(volatile struct pd_values *v){
    float proportion =  v->err                * v->kp;
    float derivative = (v->err - v->err_prev) * v->kd;

    return proportion + derivative;
}

/* killswitch */
ISR(TIMER3_COMPA_vect) {
    reset();
}

void ks_init(void) {
    /* enable interrupt on ocr3 match */
    TIMSK3 = (1<<OCIE3A);

    TCCR3A = 0;

    /* set prescaling */
    TCCR3B = (1<<WGM32)|(1<<CS31)|(1<<CS30);

    KS_OCR = KS_TOP;
    KS_TCNT = 0;
}

void pwm_init(void) {
    /* Initialize to phase and frequency correct PWM */
    TCCR1A |= (1<<COM1A1)|(1<<COM1B1);
    TCCR1B |= (1<<WGM13)|(1<<CS11);

    ICR1 = PWM_TOP;

    /* set outputs */
    DDRD |= (1<<PD4)|(1<<PD5);
}

int main(void) {
    ks_init();
    pwm_init();
    spi_init(0);

    reset();

    sei();

    while (1) {
        ctrl_val_t value_retrieved;
        uint8_t command = spi_accept((uint8_t*)&value_retrieved);

        if (command == BB_INVALID) {
            continue; /* synchronize */
        } else {
            KS_TCNT = 0; /* reset killswitch */
        }
        
        volatile struct pd_values *pdv = (command & BF_VEL_ROT) ? &vel : &rot;

        if (command & BF_WRITE) {
            if (command & BF_MOD_REG) {
                ctrl_val_t value_new;

                if (command & BF_ERR_VAL) {
                    cli();
                    pdv->err_prev = pdv->err;
                    pdv->err = value_retrieved;
                    value_new = pd_ctrl(pdv);
                    sei();
                } else {
                    value_new = value_retrieved;
                }

                float min = (command & BF_VEL_ROT) ? VEL_MIN : ROT_MIN;
                float max = (command & BF_VEL_ROT) ? VEL_MAX : ROT_MAX;
                float min_neg = (command & BF_VEL_ROT) ? VEL_MIN_NEG : ROT_MIN;
                float max_neg = (command & BF_VEL_ROT) ? VEL_MAX_NEG : ROT_MAX;

                /* limit value to valid interval */
                value_new = MIN(MAX(-1, value_new), 1);

                float duty_scaler;
                if (value_new == 0) {
                    duty_scaler = 0;
                } else if (value_new > 0) {
                    duty_scaler = (max-min)*value_new + min;
                } else {
                    duty_scaler = (max_neg-min_neg)*value_new - min_neg;
                }

                float duty = DUTY_NEUTRAL + duty_scaler*(DUTY_MAX-DUTY_NEUTRAL);

                if (command & BF_VEL_ROT) {
                    OCR_VEL = duty*PWM_TOP; 
                } else {
                    OCR_ROT = duty*PWM_TOP; 
                }
            } else {
                volatile ctrl_val_t *dst =
                    (command & BF_KP_KD) ? &pdv->kp : &pdv->kd;
                cli();
                *dst = value_retrieved;
                sei();
            }
        } else if (command == BBC_RST) {
            reset();
        }
    }
}
