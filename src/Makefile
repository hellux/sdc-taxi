OBJDIR = build

CXXC = g++
CC = gcc
CFLAGS = -Wall -g `pkg-config --cflags opencv`

AVRCC = avr-gcc
AVRCFLAGS = -mmcu=atmega1284p -Wall -static -Og

COMM_SRC_CPP_FILES = img_proc.cpp
COMM_SRC_C_FILES = server.c bus.c
COMM_MAIN = comm/main
COMM_LFLAGS = -lpthread -lm `pkg-config --libs opencv`
COMM_SRC_C = $(addprefix comm/, $(COMM_SRC_C_FILES))
COMM_SRC_CPP = $(addprefix comm/ip/, $(COMM_SRC_CPP_FILES))
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

# ignore same-named folders
.PHONY: comm sens ctrl 

## build targets

# replace main example: make comm main=dbg/server
comm: init $(COMM_OBJS) $(if $(main),$(main),$(COMM_MAIN)).o
	$(eval main=$(if $(main),$(main),$(COMM_MAIN)))
	$(CXXC) $(main).o $(COMM_OBJS) $(CFLAGS) $(COMM_LFLAGS) -o $(OBJDIR)/b$@

sens: init $(SENS_OBJS)
	$(AVRCC) $(SENS_OBJS) $(AVRCFLAGS) $(SENS_LFLAGS) -o $(OBJDIR)/b$@

ctrl: init $(CTRL_OBJS)
	$(AVRCC) $(CTRL_OBJS) $(AVRCFLAGS) $(CTRL_LFLAGS) -o $(OBJDIR)/b$@

clean:
	rm -rf $(OBJDIR)

## helper rules

init:
	mkdir -p $(OBJDIR)
	mkdir -p $(OBJDIR)/comm
	mkdir -p $(OBJDIR)/comm/ip
	mkdir -p $(OBJDIR)/avr

# compile objects for comm c files
$(OBJDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

# compile objects for comm cpp files (only ip files)
$(OBJDIR)/comm/ip/%.o: comm/ip/%.cpp
	$(CXXC) -c $(CFLAGS) $< -o $@

# compile objects for avr c files
$(OBJDIR)/avr/%.o: avr/%.c
	$(AVRCC) -c $(AVRCFLAGS) $< -o $@
