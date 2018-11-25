BINDIR = bin

PI_REMOTE = pi@172.20.10.4

CFLAGS=-std=c99 -Wall 
AVRCFLAGS = -mmcu=atmega1284p -static -DF_CPU=16000000 -Wall -Og

all: avr.prog comm.remote

comm.pi: CFLAGS += -lwiringPi -DPI
comm:
	mkdir -p $(BINDIR)
	gcc $(CFLAGS) comm.c common.c -o $(BINDIR)/comm
comm.remote:
	scp -rp comm.c avr.c common.c common.h Makefile $(PI_REMOTE):packloss
	ssh $(PI_REMOTE) -t 'cd packloss; make comm.pi'

avr:
	mkdir -p $(BINDIR)
	avr-gcc $(AVRCFLAGS) avr.c common.c -o $(BINDIR)/avr
avr.prog: avr
	avr-objcopy -R .fuse -R .lock -R .eeprom -O ihex $(BINDIR)/avr $(BINDIR)/avr.hex
	sudo avrdude -C -c atmelice -p m1284p -U flash:w:$(BINDIR)/avr.hex

clean:
	rm -rf $(BINDIR)
