OBJDIR = build
BINDIR = $(OBJDIR)/bin

CC = gcc
CFLAGS = -Wall -g -std=c99

CXXC = g++
CXXFLAGS = -Wall `pkg-config --cflags opencv`

AVRCC = avr-gcc
AVRARCH = -mmcu=atmega1284p
AVRCFLAGS = -Wall -static -Og

PROG_CONF = prog.conf
PROG_DBG = jtag1
PROG_PNO = m1284p

COMM_LFLAGS = -lpthread -lm `pkg-config --libs opencv`
COMM_SRC_C = $(shell find comm -name '*.c' -not -name 'main.c' | sed "s,\./,,")
COMM_SRC_CPP = $(shell find -name '*.cpp' | sed "s,\./,,")
COMM_OBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(COMM_SRC_C)) \
		    $(patsubst %.cpp, $(OBJDIR)/%.o, $(COMM_SRC_CPP))

SENS_SRC_FILES = sens.c
SENS_LFLAGS =
SENS_SRC = $(addprefix avr/, $(SENS_SRC_FILES))
SENS_OBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(SENS_SRC))

CTRL_SRC_FILES = ctrl.c
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

# program via debugger
%.prog: init $(BINDIR)/%.hex
	avrdude -C +$(PROG_CONF) -c $(PROG_DBG) -p $(PROG_PNO) -U flash:w:$<

# program via linux GPIO interface (on RPi)
# (use predefined programmers gpio_X from conf file)
%.gpio: init $(BINDIR)/%.hex
	$(MAKE) $*.prog PROG_DBG=gpio_$*

clean:
	rm -rf $(OBJDIR)
	rm -rf $(BINDIR)

init:
	mkdir -p $(OBJDIR)
	mkdir -p $(OBJDIR)/dbg
	mkdir -p $(OBJDIR)/comm
	mkdir -p $(OBJDIR)/comm/ip
	mkdir -p $(OBJDIR)/avr
	mkdir -p $(BINDIR)

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
