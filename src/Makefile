COMM_OBJS_SRV = comm/server.c
COMM_OBJS_BUS = comm/bus.c
COMM_OBJS_IP = comm/img_proc.cpp
COMM_OBJS_C = $(COMM_OBJS_BUS) $(COMM_OBJS_SRV)
COMM_MAIN = comm/main.c 
COMM_SRV = dbg/server.c
COMM_BUS = dbg/bus.c
COMM_IP = dbg/ip.c
COMM_LFLAGS = -lpthread # -lm -ldl `pkg-config --libs --cflags opencv`

SENS_OBJS = avr/sens.c
SENS_LFLAGS =

CTRL_OBJS = avr/ctrl.c
CTRL_LFLAGS =

CXXC = g++
CC = gcc
CFLAGS = -Wall -g

# cross compilation
AVR_CFLAGS = -Wall -static -Og
AVR_ARCH = -mmcu=atmega1284p
AVR_CC = avr-gcc

# ignore same-named folders
.PHONY: comm sens ctrl 

comm: $(COMM_OBJS)
	make clean
	$(CXXC) -c $(COMM_OBJS_IP) $(CFLAGS)
	$(CC) -c $(COMM_MAIN) $(COMM_OBJS_C) $(CFLAGS) $(COMM_LFLAGS)
	$(CXXC) -o bcomm *.o $(CFLAGS) $(COMM_LFLAGS)
	make clean

comm_srv: $(COMM_SRV) $(COMM_OBJS)
	$(CC) -o bcomm_srv $(COMM_SRV) $(COMM_OBJS_SRV) $(CFLAGS) $(COMM_LFLAGS)

comm_bus: $(COMM_BUS) $(COMM_OBJS)
	$(CC) -o bcomm_bus $(COMM_BUS) $(COMM_OBJS_BUS) $(CFLAGS) $(COMM_LFLAGS)

comm_ip: $(COMM_BUS) $(COMM_OBJS)
	make clean
	$(CXXC) -c $(COMM_OBJS_IP) $(CFLAGS) 
	$(CC) -c $(COMM_IP) $(CFLAGS) $(COMM_LFLAGS)
	$(CXXC) -o bcomm_ip *.o $(CFLAGS) $(COMM_LFLAGS)
	make clean

sens: $(SENS_OBJS)
	$(AVR_CC) $(AVR_ARCH) $(SENS_OBJS) $(AVR_CFLAGS) $(SENS_LFLAGS) -o bsens

ctrl: $(CTRL_OBJS)
	$(AVR_CC) $(AVR_ARCH) $(CTRL_OBJS) $(AVR_CFLAGS) $(CTRL_LFLAGS) -o bctrl

clean:
	rm -f *.o
