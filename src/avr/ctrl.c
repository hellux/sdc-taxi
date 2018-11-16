#include "bus.h"
#include "lcd.h"
#include "jtag.h"

#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#ifdef SIM
#include "../dbg/sim/ctrl_header.h"
#endif

#define PWM_T 0.020
#define PWM_PSC 16
#define PWM_TOP F_CPU*PWM_T/PWM_PSC

typedef struct {
    float kp;           //Constant Pro
    float kd;           //Constant Der
    float err;          //Error
    float last_err;     //Previous error
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
    uint8_t data;

    //Run SPI Recive function 
    data = spi_tranceive(0);

    //First byte = FF if constant
    if(data == 0xFF){
        //Recieve all the bytes needed for the two constants (Kd, Kp)
        //data = spi_tranceive(0);
    }
    //First byte = 00 if error
    else if(data == 0x00){
        //Recieve the two bytes needed for error
        //data = spi_tranceive(0);
    }

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
    float duty_vel = 0;
    float duty_rad = 0;

    pwm_init();
    //spi_init_slave();
    //init_jtagport();
    //init_lcdports();
    
    //int counter= 0;

    //Enable global interrupts
    sei();

    while(1){
        
        //duty_vel = pd_ctrl(&vel);
        //duty_rad = pd_ctrl(&rad);
        
        duty_vel = 0.075 * PWM_TOP;
        OCR1B = duty_vel;
        //duty_rad = 0.078 * PWM_TOP;

        /*                
        if(counter %4 == 1){
            duty_rad = 0.077 * PWM_TOP;
        }
        else if(counter %4 == 2){
            duty_rad = 0.075 * PWM_TOP;
        }
        else if(counter %4 == 3){
            duty_rad = 0.073 * PWM_TOP;
        }
        else if(counter %4 == 0){
            duty_rad = 0.075 * PWM_TOP;
        }
        counter++;
        */
        //_delay_ms(2000);        
         
        duty_rad = 0.078 * PWM_TOP;
        OCR1A = duty_rad;
        _delay_ms(2000);

        duty_rad = 0.075 * PWM_TOP;
        OCR1A = duty_rad;
        _delay_ms(2000);

        duty_rad = 0.072 * PWM_TOP;
        OCR1A = duty_rad;
        _delay_ms(2000);

        duty_rad = 0.075 * PWM_TOP;
        OCR1A = duty_rad;
        _delay_ms(2000);        
        
        //OCR1A = duty_rad;
        
    }
    
    return 0;
}
