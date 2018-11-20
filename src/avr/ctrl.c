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

#define PWM_T 0.020
#define PWM_PSC 16
#define PWM_TOP F_CPU*PWM_T/PWM_PSC

typedef struct {
    ctrl_const_t kp;            //Constant Pro
    ctrl_const_t kd;            //Constant Der
    ctrl_err_t err;             //Error
    ctrl_err_t last_err;        //Previous error
} pd_values_t;

volatile pd_values_t vel;
volatile pd_values_t rad;

void pwm_init(){
    //Initialize to phase and frequency correct PWM
    TCCR1A |= (1<<COM1A1)|(1<<COM1B1);
    TCCR1B |= (1<<WGM13)|(1<<CS11);
    
    //Set TOP value
    ICR1 = PWM_TOP;

    //Set PD4, PD5 to outputs
    DDRD |= (1<<PD4)|(1<<PD5);
}

ISR(SPI_STC_vect){
    cli();
    uint8_t command; 
    spi_tranceive(&command, sizeof(command));

    if(command == BCB_ROT){
        float data; 
        spi_tranceive((uint8_t*)&data, sizeof(data));
        if(data < 0.05){
            OCR1A = 0.05 * PWM_TOP; 
        }
        else if(data > 0.10){
            OCR1A = 0.10 * PWM_TOP; 
        }
        else{
            OCR1A = data * PWM_TOP; 
        }
    }

    else if(command == BCB_VEL){        
        float data; 
        spi_tranceive((uint8_t*)&data, sizeof(data));
        if(data < 0.05){  
          OCR1B = 0.05 * PWM_TOP; 
        }
        else if(data > 0.079){  
          OCR1B = 0.079 * PWM_TOP; 
        }
        else{
          OCR1B = data * PWM_TOP; 
        }
    }
   /* 
    else if(command == BCB_VEL_KP){
        float data; 
        spi_tranceive((uint8_t*)&data, sizeof(data));
        vel->kp = data;
    }
    else if(command == BCB_VEL_KD){
        float data; 
        spi_tranceive((uint8_t*)&data, sizeof(data));
        vel->kd = data;
    }
    else if(command == BCB_ROT_KP){
        float data; 
        spi_tranceive((uint8_t*)&data, sizeof(data));
        rad->kp = data;
    }
    else if(command == BCB_ROT_KP){
        float data; 
        spi_tranceive((uint8_t*)&data, sizeof(data));
        rad->kp = data;
    }
    */
    sei();
}

float pd_ctrl(volatile pd_values_t *v){
    float proportion;
    float derivative;
    proportion = v->err                 * v->kp;
    derivative = (v->err - v->last_err) * v->kd;
    v->last_err = v->err;

    return proportion + derivative;
}

int main(int argc, char* args[]) {
    //float duty_vel = 0;
    //float duty_rad = 0;

    pwm_init();
    spi_init_slave();
    init_jtagport();
    //init_lcdports();
   
    //Enable global interrupts
    sei();

    while(1){
        
        //duty_vel = pd_ctrl(&vel);
        //duty_rad = pd_ctrl(&rad);
        
        /*
        duty_vel = 0.078 * PWM_TOP;
        OCR1B = duty_vel;
        
        duty_rad = 0.050 * PWM_TOP;
        OCR1A = duty_rad;
        _delay_ms(3000);
        */
    }
    
    return 0;
}
