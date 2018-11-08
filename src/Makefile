OBJS_COMM_C = comm/main.c comm/bus.c comm/server.c
OBJS_COMM_CPP = comm/img_proc.cpp
OBJS_SENS = sens/main.c
OBJS_CTRL = ctrl/main.c

LFLAGS_COMM = -lpthread # -lm -ldl `pkg-config --libs --cflags opencv`
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
	g++ -c $(OBJS_COMM_CPP)
	$(CC) -c $(OBJS_COMM_C) $(CFLAGS) $(LFLAGS_COMM)
	g++ -o bcomm *.o $(LFLAGS_COMM)

sens: $(OBJS_SENS)
	$(AVR_CC) $(AVR_ARCH) $(OBJS_SENS) $(AVR_CFLAGS) $(LFLAGS_SENS) -o bsens

ctrl: $(OBJS_CTRL)
	$(AVR_CC) $(AVR_ARCH) $(OBJS_CTRL) $(AVR_CFLAGS) $(LFLAGS_CTRL) -o bctrl

clean:
	rm -f *.o
