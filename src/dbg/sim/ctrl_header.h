#include "simavr/avr_mcu_section.h"
AVR_MCU(F_CPU, "atmega1284p"); 

const struct avr_mmcu_vcd_trace_t _mytrace[]  _MMCU_ = {
    { AVR_MCU_VCD_SYMBOL("DDRD"), .what = (void*)&DDRD, },
    { AVR_MCU_VCD_SYMBOL("OCR1A"), .what = (void*)&OCR1A, },
    { AVR_MCU_VCD_SYMBOL("OCR1B"), .what = (void*)&OCR1B, },
    { AVR_MCU_VCD_SYMBOL("PORTD"), .what = (void*)&PORTD, },
    { AVR_MCU_VCD_SYMBOL("TCNT1H"), .what = (void*)&TCNT1H, },
    { AVR_MCU_VCD_SYMBOL("TCNT1L"), .what = (void*)&TCNT1L, },
    { AVR_MCU_VCD_SYMBOL("ICR1H"), .what = (void*)&ICR1H, },
    { AVR_MCU_VCD_SYMBOL("ICR1L"), .what = (void*)&ICR1L, },
};
