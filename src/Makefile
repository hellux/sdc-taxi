OBJDIR = build
BINDIR = $(OBJDIR)/bin

CC = gcc
CFLAGS = -std=c99 -D_POSIX_C_SOURCE=200809L -DIP
CFLAGS.debug = -Wall -Og -ggdb -Og
CFLAGS.release = -O3

PI_REMOTE = $(shell sh ip.sh)

CXXC = g++
CXXFLAGS = -DCPP `pkg-config --cflags opencv`
CXXFLAGS.debug = -Wall -ggdb -Og
CXXFLAGS.release = -O3

AVRCC = avr-gcc
AVRARCH = -mmcu=atmega1284p
AVRCFLAGS = -static -DF_CPU=16000000 -Wall -Og -DDEBUG

PROG_CONF = prog.conf
PROG_DBG = atmelice
PROG_PNO = m1284p

COMM_LFLAGS = -lpthread -lm `pkg-config --libs opencv`
COMM_BIN = comm
COMM_SRC_C_FILES = main.c server.c spi.c bus.c protocol.c objective.c
COMM_SRC_C = $(addprefix comm/, $(COMM_SRC_C_FILES))
COMM_SRC_CPP = $(shell find -name '*.cpp' | sed "s,\./,,")
COMM_OBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(COMM_SRC_C)) \
		    $(patsubst %.cpp, $(OBJDIR)/%.o, $(COMM_SRC_CPP))

SENS_SRC_FILES = sens.c bus.c protocol.c lcd.c
SENS_LFLAGS =
SENS_SRC = $(addprefix avr/, $(SENS_SRC_FILES))
SENS_OBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(SENS_SRC))

CTRL_SRC_FILES = ctrl.c bus.c protocol.c lcd.c
CTRL_LFLAGS =
CTRL_SRC = $(addprefix avr/, $(CTRL_SRC_FILES))
CTRL_OBJS = $(patsubst %.c, $(OBJDIR)/%.o, $(CTRL_SRC))

.PHONY: comm
.SILENT: init

# communication module
comm.gdb: CXXFLAGS += $(CXXFLAGS.debug)
comm.gdb: CFLAGS += $(CFLAGS.debug)
comm.gdb: $(BINDIR)/comm
	gdb -ex run --args $(BINDIR)/$(COMM_BIN) 127.0.0.1
comm.sample.visual: CFLAGS += -DVISUAL
comm.sample.visual: CXXFLAGS += -DVISUAL
comm.sample.visual: comm.sample
comm.sample: CFLAGS += $(CFLAGS.debug) -DRECORD -DSAMPLE
comm.sample: CXXFLAGS += $(CXXFLAGS.debug) -DRECORD -DSAMPLE
comm.sample: $(BINDIR)/comm
	$(BINDIR)/$(COMM_BIN) 127.0.0.1 && mpv "opencv.avi"
comm.pi.release: CXXFLAGS += $(CXXFLAGS.release)
comm.pi.release: CFLAGS += $(CFLAGS.release)
comm.pi.release: comm.pi
comm.pi.record: CFLAGS += -DRECORD -O3
comm.pi.record: CXXFLAGS += -DRECORD -O3
comm.pi.record: comm.pi
comm.pi.visual: CFLAGS += -DVISUAL
comm.pi.visual: CXXFLAGS += -DVISUAL
comm.pi.visual: comm.pi
comm.pi.debug: CXXFLAGS += $(CXXFLAGS.debug)
comm.pi.debug: CFLAGS += $(CFLAGS.debug)
comm.pi.debug: comm.pi
comm.pi: CFLAGS += -DPI
comm.pi: $(BINDIR)/comm
	$(BINDIR)/comm $(shell ip -br addr show dev wlan0 | \
		tr -s ' ' | cut -d' ' -f3 | cut -d'/' -f1)
comm.ip: CXXFLAGS += $(CXXFLAGS.debug) -DVISUAL -DRECORD
comm.ip: $(addprefix $(OBJDIR)/, dbg/ip.o comm/ip/img_proc.o)
	$(CXXC) $? $(COMM_LFLAGS) $(CXXFLAGS) -o $(BINDIR)/comm_ip
comm.remote.clean:
	ssh $(PI_REMOTE) -t 'cd module; make clean'
comm.push:
	scp -rp comm spi Makefile $(PI_REMOTE):module
comm.remote.record: comm.push
	ssh -XY $(PI_REMOTE) -t 'cd module; make comm.pi.record'
comm.remote.visual: comm.push
	ssh -XY $(PI_REMOTE) -t 'cd module; make comm.pi.visual'
comm.remote: comm.push
	ssh -XY $(PI_REMOTE) -t 'cd module; make comm.pi'
comm.pull_video:
	ssh $(PI_REMOTE) -t 'cd module; ffmpeg -i opencv.avi -vcodec libx265 -crf 24 lowrate.mp4'
	scp $(PI_REMOTE):module/lowrate.mp4 .
comm.plot_vel:
	scp $(PI_REMOTE):module/vel.dat .
	gnuplot -p -e 'plot "vel.dat" with lines'

# avr modules
sens: $(BINDIR)/sens
ctrl: $(BINDIR)/ctrl
%.prog: $(BINDIR)/%.hex
	sudo avrdude -C +$(PROG_CONF) -c $(PROG_DBG) -p $(PROG_PNO) -U flash:w:$(BINDIR)/$*.hex
%.erase:
	sudo avrdude -c $(PROG_DBG) -p $(PROG_PNO) -e
%.gpio: $(BINDIR)/%.hex
	$(MAKE) $*.prog PROG_DBG=gpio_$*

init:
	mkdir -p $(OBJDIR)
	mkdir -p $(OBJDIR)/dbg
	mkdir -p $(OBJDIR)/comm
	mkdir -p $(OBJDIR)/comm/ip
	mkdir -p $(OBJDIR)/avr
	mkdir -p $(BINDIR)
clean:
	rm -rf $(OBJDIR)
	rm -rf $(BINDIR)

# compile objects for comm c files
$(OBJDIR)/%.o: %.c | init
	$(CC) -c $(CFLAGS) $< -o $@

# compile objects for comm cpp files (only ip files)
$(OBJDIR)/comm/ip/%.o: comm/ip/%.cpp | init
	$(CXXC) -c $(CXXFLAGS) $< -o $@

# link comm to binary
$(BINDIR)/comm: $(COMM_OBJS)
	$(CXXC) $(COMM_OBJS) $(COMM_LFLAGS) -o $@

# compile objects for avr c files
$(OBJDIR)/avr/%.o: avr/%.c | init
	$(AVRCC) -c $(AVRARCH) $(AVRCFLAGS) $< -o $@

# link sens to binary
$(BINDIR)/sens: AVRCFLAGS += -DSLAVE=0
$(BINDIR)/sens: $(SENS_OBJS)
	$(AVRCC) $(SENS_OBJS) $(AVRARCH) $(SENS_LFLAGS) -o $@

# link ctrl to binary
$(BINDIR)/ctrl: AVRCFLAGS += -DSLAVE=1
$(BINDIR)/ctrl: $(CTRL_OBJS)
	$(AVRCC) $(CTRL_OBJS) $(AVRARCH) $(CTRL_LFLAGS) -o $@

# create hex from binary
%.hex: %
	avr-objcopy -R .fuse -R .lock -R .eeprom -O ihex $< $@
