#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#include "protocol.h"
#include "bus.h"

#define MIN(A, B) ((A) < (B) ? (A) : (B))
#define MAX(A, B) ((A) > (B) ? (A) : (B))

/* PWM */
#define VEL_K 0.062
#define DUTY_MAX 0.10
#define DUTY_NEUTRAL 0.075
#define DUTY_MIN 0.050
#define VEL_MIN -0.2
#define VEL_NEUTRAL 0.06
#define VEL_MAX 0.5
#define ROT_MIN -1
#define ROT_MAX 1

#define PWM_T 0.020
#define PWM_PSC 16
#define PWM_TOP F_CPU*PWM_T/PWM_PSC

#define OCR_VEL OCR1B
#define OCR_ROT OCR1A

/* Cohen Coon tuning */
#define CC_A 1.3
#define CC_B 1.32
#define CC_K (CC_B/CC_A)
#define CC_T0 5.3
#define CC_T2 5.71
#define CC_T3 5.78
#define CC_T1 ((CC_T2-.693*CC_T3)/.306)
#define CC_T (CC_T3-CC_T1)
#define CC_TDEL (CC_T1-CC_T0)
#define CC_R (CC_TDEL/CC_T)
#define CC_KP (1/CC_K/CC_R*(4.0/3.0+CC_R/4.0))
#define CC_KI (CC_TDEL*(32+6.0*CC_R)/(13+8.0*CC_R))
#define CC_KD (CC_TDEL*(4.0/(11+2.0*CC_R)))

/* Zieger Nichols */
#define ZN_KU 0
#define ZN_TU 0
#define ZN_KP (0.6*ZN_KU)
#define ZN_KI (1.2*ZN_KU/ZN_TU)
#define ZN_KD (3.0*ZN_KU/ZN_TU/40.0)

#define VEL_KP 0.5
#define VEL_KI 0.03
#define VEL_KD 0.1

#define ROT_KP 2.5
#define ROT_KI 0
#define ROT_KD 1.9

/* killswitch */
#define KS_OCR OCR3A
#define KS_TCNT TCNT3

#define KS_T 0.2
#define KS_PSC 64
#define KS_TOP F_CPU*KS_T/KS_PSC

#define CTRL_SLAVE 1

struct pd_values {
    ctrl_val_t kp;
    ctrl_val_t ki;
    ctrl_val_t kd;

    ctrl_val_t err;
    ctrl_val_t err_prev;
    ctrl_val_t err_sum;
};

const struct pd_values PD_EMPTY = {0};

volatile struct pd_values vel;
volatile struct pd_values rot;

void reset(void) {
    cli();
    vel = PD_EMPTY;
    vel.kp = VEL_KP;
    vel.ki = VEL_KI;
    vel.kd = VEL_KD;
    rot = PD_EMPTY;
    rot.kp = ROT_KP;
    rot.ki = ROT_KI;
    rot.kd = ROT_KD;
    OCR_VEL = DUTY_NEUTRAL*PWM_TOP;
    OCR_ROT = DUTY_NEUTRAL*PWM_TOP;
    sei();
}

float pd_ctrl(volatile struct pd_values *v){
    float proportion =  v->err                * v->kp;
    float integration = v->err_sum            * v->ki;
    float derivative = (v->err - v->err_prev) * v->kd;

    return proportion + integration + derivative;
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
    /* ks_init(); */
    pwm_init();
    spi_init(0);

    reset();

    /* sei(); */

    while (1) {
        ctrl_val_t value_retrieved;
        uint8_t command = spi_accept((uint8_t*)&value_retrieved,
                                     SPI_OUTSIDE_ISR, CTRL_SLAVE);

        if (command == BB_INVALID) {
            continue;
        } else {
            KS_TCNT = 0;
        }
        
        volatile struct pd_values *pdv = (command & BF_VEL_ROT) ? &vel : &rot;

        if (command & BF_WRITE) {
            if (command & BF_MOD_REG) {
                ctrl_val_t value_new;

                if (command & BF_ERR_VAL) {
                    cli();
                    pdv->err_prev = pdv->err;
                    pdv->err = value_retrieved;
                    pdv->err_sum += value_retrieved;
                    value_new = pd_ctrl(pdv);
                    sei();
                } else {
                    cli();
                    pdv->err = 0;
                    pdv->err_sum = 0;
                    sei();

                    if (command & BF_VEL_ROT) {
                        if (value_retrieved <= 0) {
                            value_new = value_retrieved;
                        } else {
                            value_new = VEL_NEUTRAL+value_retrieved*VEL_K;
                        }
                    } else {
                        value_new = value_retrieved;
                    }
                }

                float min = (command & BF_VEL_ROT) ? VEL_MIN : ROT_MIN;
                float max = (command & BF_VEL_ROT) ? VEL_MAX : ROT_MAX;

                value_new = MIN(MAX(min, value_new), max);

                float duty = DUTY_NEUTRAL + value_new*(DUTY_MAX-DUTY_NEUTRAL);

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
