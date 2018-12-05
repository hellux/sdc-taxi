BINDIR = bin

PI_REMOTE = $(shell sh ../../ip.sh)

AVR_MAIN = avr.c
CFLAGS=-std=c99 -Wall 
AVRCFLAGS = -mmcu=atmega1284p -static -DF_CPU=16000000 -Wall -Og

all: clean avr.prog comm.remote

comm.pi: CFLAGS += -DPI
comm.pi:
	mkdir -p $(BINDIR)
	gcc $(CFLAGS) comm.c -o $(BINDIR)/comm
	./$(BINDIR)/comm
comm:
	mkdir -p $(BINDIR)
	gcc $(CFLAGS) comm.c -o $(BINDIR)/comm
comm.remote:
	scp -rp comm.c avr.c Makefile $(PI_REMOTE):packloss
	ssh $(PI_REMOTE) -t 'cd packloss; make clean; make comm.pi'

avr:
	mkdir -p $(BINDIR)
	avr-gcc $(AVRCFLAGS) $(AVR_MAIN) -o $(BINDIR)/avr
avr.prog: avr
	avr-objcopy -R .fuse -R .lock -R .eeprom -O ihex $(BINDIR)/avr $(BINDIR)/avr.hex
	sudo avrdude -c atmelice -p m1284p -U flash:w:$(BINDIR)/avr.hex

avr.int.prog: AVR_MAIN = avr_int.c
avr.int.prog: avr.prog

clean:
	rm -rf $(BINDIR)
