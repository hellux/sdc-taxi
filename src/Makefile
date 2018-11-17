OBJDIR = build
BINDIR = $(OBJDIR)/bin
SIMDIR = $(OBJDIR)/sim

CC = gcc
CFLAGS = -Wall -std=c99 -D_POSIX_C_SOURCE=200809L

CXXC = g++
CXXFLAGS = -Wall `pkg-config --cflags opencv`

AVRCC = avr-gcc
AVRARCH = -mmcu=atmega1284p
AVRCFLAGS = -Wall -static -Og -DF_CPU=16000000

PROG_CONF = prog.conf
PROG_DBG = atmelice
PROG_PNO = m1284p

COMM_LFLAGS = -lpthread -lm `pkg-config --libs opencv`
COMM_MAIN = comm/main
COMM_BIN = comm
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

# communication module
comm: COMM_OBJS += $(OBJDIR)/$(COMM_MAIN).o
comm: init $(OBJDIR)/$(COMM_MAIN).o $(BINDIR)/comm
comm.pi: CFLAGS += -DPI -O3
comm.pi: CXXFLAGS += -O3
comm.pi: COMM_LFLAGS += -lwiringPi
comm.pi: comm
	$(BINDIR)/comm $(shell ip -br addr show dev wlan0 | \
		tr -s ' ' | cut -d' ' -f3 | cut -d'/' -f1)
comm.server: COMM_OBJS += $(OBJDIR)/dbg/server.o
comm.server: init $(OBJDIR)/dbg/server.o $(BINDIR)/comm
comm.ip: COMM_BIN = comm_ip
comm.ip: COMM_OBJS += $(OBJDIR)/dbg/ip.o
comm.ip: init $(OBJDIR)/dbg/ip.o $(BINDIR)/comm

# avr modules
sens: init $(BINDIR)/sens
ctrl: init $(BINDIR)/ctrl
%.sim: AVRCFLAGS += -DSIM
%.sim: init $(SIMDIR)/% $(BINDIR)/%
	./$(SIMDIR)/$*
%.prog: init $(BINDIR)/%.hex
	sudo avrdude -C +$(PROG_CONF) -c $(PROG_DBG) -p $(PROG_PNO) -U flash:w:$(BINDIR)/$*.hex
%.gpio: init $(BINDIR)/%.hex
	$(MAKE) $*.prog PROG_DBG=gpio_$*

init:
	mkdir -p $(OBJDIR)
	mkdir -p $(OBJDIR)/dbg
	mkdir -p $(OBJDIR)/sim
	mkdir -p $(OBJDIR)/comm
	mkdir -p $(OBJDIR)/comm/ip
	mkdir -p $(OBJDIR)/avr
	mkdir -p $(BINDIR)
clean:
	rm -rf $(OBJDIR)
	rm -rf $(BINDIR)

# compile simulator bench
$(SIMDIR)/%: dbg/sim/%.c
	$(CC) -lelf -lsimavr $< -o $@

# compile objects for comm c files
$(OBJDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

# compile objects for comm cpp files (only ip files)
$(OBJDIR)/comm/ip/%.o: comm/ip/%.cpp
	$(CXXC) -c $(CXXFLAGS) $< -o $@

# TODO fix multiple mains
# link comm to binary
$(BINDIR)/comm: $(COMM_OBJS) $(OBJDIR)/$(COMM_MAIN).o
	$(CXXC) $(COMM_OBJS) $(COMM_LFLAGS) -o $(BINDIR)/$(COMM_BIN)

# compile objects for avr c files
$(OBJDIR)/avr/%.o: avr/%.c
	$(AVRCC) -c $(AVRARCH) $(AVRCFLAGS) $< -o $@

# link sens to binary
$(BINDIR)/sens: $(SENS_OBJS)
	$(AVRCC) $(SENS_OBJS) $(AVRARCH) $(SENS_LFLAGS) -o $@

# link ctrl to binary
$(BINDIR)/ctrl: $(BINDIR) $(CTRL_OBJS)
	$(AVRCC) $(CTRL_OBJS) $(AVRARCH) $(CTRL_LFLAGS) -o $@

# create hex from binary
%.hex: %
	avr-objcopy -R .fuse -R .lock -R .eeprom -O ihex $< $@
