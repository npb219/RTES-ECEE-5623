INCLUDE_DIRS = 
LIB_DIRS = 
CC = gcc

CDEFS = 
CFLAGS = -O3 -g -Wcpp $(INCLUDE_DIRS) $(CDEFS)
LIBS = -lpthread -lrt

HFILES = capturelib.h
CFILES = synchronome.c capturelib.c

OBJS = ${CFILES:.c=.o}

all: synchronome

clean:
	-rm -f *.o *.d synchronome
	-rm -f frames/*

distclean:
	-rm -f *.o *.d

# Link all object files into the final executable
synchronome: $(OBJS)
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $(OBJS) $(LIBS)

depend:

# Rule for compiling .c to .o
.c.o:
	$(CC) $(CFLAGS) -c $<
