OBJDIR = build
BINDIR = $(OBJDIR)/bin
SIMDIR = $(OBJDIR)/sim

CC = gcc
CFLAGS = -Wall -g -std=c99

CXXC = g++
CXXFLAGS = -Wall `pkg-config --cflags opencv`

AVRCC = avr-gcc
AVRARCH = -mmcu=atmega1284p
AVRCFLAGS = -Wall -static -Og -DF_CPU=16000000
DSIM =

PROG_CONF = prog.conf
PROG_DBG = jtag1
PROG_PNO = m1284p

COMM_LFLAGS = -lpthread -lm `pkg-config --libs opencv` -lwiringPi
COMM_SRC_C = $(shell find comm -name '*.c' -not -name 'main.c' | sed "s,\./,,")
COMM_SRC_CPP = $(shell find -name '*.cpp' | sed "s,\./,,")
COMM_OBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(COMM_SRC_C)) \
		    $(patsubst %.cpp, $(OBJDIR)/%.o, $(COMM_SRC_CPP))

SENS_SRC_FILES = sens.c bus.c jtag.c lcd.c
SENS_LFLAGS =
SENS_SRC = $(addprefix avr/, $(SENS_SRC_FILES))
SENS_OBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(SENS_SRC))

CTRL_SRC_FILES = ctrl.c bus.c jtag.c lcd.c
CTRL_LFLAGS =
CTRL_SRC = $(addprefix avr/, $(CTRL_SRC_FILES))
CTRL_OBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(CTRL_SRC))

.PHONY: comm
.SILENT: init

# main.c can be replaced with another file for debugging
# 	for example, to use dbg/server.c instead of comm/main.c:
# 		make comm main=dbg/server
comm: init $(BINDIR)/comm

sens: init $(BINDIR)/sens

ctrl: init $(BINDIR)/ctrl

comm.server: init
	$(CC) dbg/server.c comm/server.c -o $(BINDIR)/srv
	./$(BINDIR)/srv

comm.ip: init
	$(CC) -c dbg/ip.c -o $(OBJDIR)/dbg/ip.o
	$(CXXC) -O3 -c $(CXXFLAGS) comm/ip/img_proc.cpp -o $(OBJDIR)/comm/ip/img_proc.o
	$(CXXC) $(OBJDIR)/comm/ip/img_proc.o $(OBJDIR)/dbg/ip.o -lm `pkg-config --libs opencv` -o $(BINDIR)/ip
	./$(BINDIR)/ip

# simulate avr
%.sim: SIM=-DSIM
%.sim: init $(SIMDIR)/% $(BINDIR)/%
	./$(SIMDIR)/$*

# program via debugger
%.prog: init $(BINDIR)/%.hex
	avrdude -C +$(PROG_CONF) -c $(PROG_DBG) -p $(PROG_PNO) -U flash:w:$<

# program via linux GPIO interface (on RPi)
# (use predefined programmers gpio_X from conf file)
%.gpio: init $(BINDIR)/%.hex
	$(MAKE) $*.prog PROG_DBG=gpio_$*

comm.run: comm
	$(BINDIR)/comm $(shell ip -br addr show dev wlan0 | tr -s ' ' | cut -d' ' -f3 | cut -d'/' -f1)

clean:
	rm -rf $(OBJDIR)
	rm -rf $(BINDIR)

init:
	mkdir -p $(OBJDIR)
	mkdir -p $(OBJDIR)/dbg
	mkdir -p $(OBJDIR)/sim
	mkdir -p $(OBJDIR)/comm
	mkdir -p $(OBJDIR)/comm/ip
	mkdir -p $(OBJDIR)/avr
	mkdir -p $(BINDIR)

# compile simulator bench
$(SIMDIR)/%: dbg/sim/%.c
	$(CC) -lelf -lsimavr $< -o $@

# compile objects for comm c files
$(OBJDIR)/%.o: %.c | $(OBJDIR)/comm/
	$(CC) -c $(CFLAGS) $< -o $@

# compile objects for comm cpp files (only ip files)
$(OBJDIR)/comm/ip/%.o: comm/ip/%.cpp
	$(CXXC) -c $(CXXFLAGS) $< -o $@

# link comm to binary
$(BINDIR)/comm: $(COMM_OBJS) $(OBJDIR)/$(if $(main),$(main),comm/main).o
	$(eval main=$(if $(main),$(main),comm/main))
	$(CXXC) $(OBJDIR)/$(main).o $(COMM_OBJS) $(COMM_LFLAGS) -o $@

# compile objects for avr c files
$(OBJDIR)/avr/%.o: avr/%.c
	$(AVRCC) -c $(AVRARCH) $(AVRCFLAGS) $(SIM) $< -o $@

# link sens to binary
$(BINDIR)/sens: $(SENS_OBJS)
	$(AVRCC) $(SENS_OBJS) $(AVRARCH) $(SENS_LFLAGS) -o $@

# link ctrl to binary
$(BINDIR)/ctrl: $(BINDIR) $(CTRL_OBJS)
	$(AVRCC) $(CTRL_OBJS) $(AVRARCH) -I/home/noah/build-repos/simavr/include $(CTRL_LFLAGS) -o $@

# create hex from binary
%.hex: %
	avr-objcopy -R .fuse -R .lock -R .eeprom -O ihex $< $@
