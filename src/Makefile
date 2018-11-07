OBJS_COMM = comm/main.c
OBJS_SENS = sens/main.c
OBJS_CTRL = ctrl/main.c

LFLAGS_COMM = -lm -ldl `pkg-config --libs --cflags opencv`
LFLAGS_SENS =
LFLAGS_CTRL =

CC = gcc
CFLAGS = -Wall

# cross compilation
AVR_CFLAGS = -Wall -static -Og
AVR_ARCH = -mmcu=atmega1284p
AVR_CC = avr-gcc

# ignore same-named folders
.PHONY: comm sens ctrl 

comm: $(OBJS_COMM)
	$(CC) $(OBJS_COMM) $(CFLAGS) $(LFLAGS_COMM) -o bcomm

sens: $(OBJS_SENS)
	$(AVR_CC) $(AVR_ARCH) $(OBJS_SENS) $(AVR_CFLAGS) $(LFLAGS_SENS) -o bsens

ctrl: $(OBJS_CTRL)
	$(AVR_CC) $(AVR_ARCH) $(OBJS_CTRL) $(AVR_CFLAGS) $(LFLAGS_CTRL) -o bctrl
