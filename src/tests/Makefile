OBJDIR = build
BINDIR = $(OBJDIR)/bin

CC = gcc
CFLAGS = -ggdb -O3 -std=c99 -D_POSIX_C_SOURCE=200809L
LFLAGS = -lpthread

SRC.proto = spi/protocol.c proto.c 
$(BINDIR)/proto: $(patsubst %.c, $(OBJDIR)/%.o, $(SRC.proto))

SRC.bus = comm/bus.c bus.c
$(BINDIR)/bus: $(patsubst %.c, $(OBJDIR)/%.o, $(SRC.bus))

.SILENT:

all: testc.proto testc.bus testpy.course
	echo "All tests passed, yeeeeeess."

testpy.%:
	python $*.py
	echo "PASS: $*"

testc.%: $(BINDIR)/%
	./$<
	echo "PASS: $*"

clean:
	rm -rf $(OBJDIR)

init:
	mkdir -p $(OBJDIR)
	mkdir -p $(OBJDIR)/spi
	mkdir -p $(OBJDIR)/comm
	mkdir -p $(BINDIR)

$(OBJDIR)/%.o: %.c | init
	$(CC) -c $(CFLAGS) $< -o $@

$(BINDIR)/%:
	$(CC) $^ $(LFLAGS) -o $@
