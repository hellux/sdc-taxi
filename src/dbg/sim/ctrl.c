#include <stdlib.h>
#include <stdio.h>
#include <libgen.h>
#include <pthread.h>

#include "simavr/sim_io.h"
#include "simavr/avr_ioport.h"
#include "simavr/sim_avr.h"
#include "simavr/avr_twi.h"
#include "simavr/sim_elf.h"
#include "simavr/sim_gdb.h"
#include "simavr/sim_vcd_file.h"

avr_t * avr = NULL;
avr_vcd_t vcd_file;

int main(int argc, char *argv[])
{
	elf_firmware_t f;
	const char *fname = "build/bin/ctrl";

	printf("Firmware pathname is %s\n", fname);
	elf_read_firmware(fname, &f);

	printf("firmware %s f=%d mmcu=%s\n", fname, (int)f.frequency, f.mmcu);

	avr = avr_make_mcu_by_name(f.mmcu);
	if (!avr) {
		fprintf(stderr, "%s: AVR '%s' not known\n", argv[0], f.mmcu);
		exit(1);
	}
	avr_init(avr);
	avr_load_firmware(avr, &f);

	avr_vcd_init(avr, "gtkwave_output.vcd", &vcd_file, 1);
	avr_vcd_add_signal(&vcd_file,
		avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('A'), IOPORT_IRQ_PIN_ALL), 8,
		"DDRD" );
	avr_vcd_add_signal(&vcd_file,
		avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('A'), IOPORT_IRQ_PIN_ALL), 8,
		"OCR1A" );
    avr_vcd_add_signal(&vcd_file, avr_get_interrupt_irq(avr, 18), 8, "PORTD" );

    avr_vcd_start(&vcd_file);

	printf( "\nDemo launching:\n");

    int time = 2;
    int n = 0;
	int state = cpu_Running;
	while ((state != cpu_Done) && (state != cpu_Crashed) && n++ < time*16000000)
		state = avr_run(avr);

    avr_vcd_stop(&vcd_file);
    avr_vcd_close(&vcd_file);
}
