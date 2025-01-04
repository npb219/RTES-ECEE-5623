INCLUDE_DIRS = 
LIB_DIRS = 
CC=gcc

CDEFS=
CFLAGS= -O0 -march=native -g $(INCLUDE_DIRS) $(CDEFS)
//CFLAGS= -O3 -msse3 -malign-double -g $(INCLUDE_DIRS) $(CDEFS)
LIBS=-lpthread -lrt

DRIVER=stripetest

HFILES= raidlib.h
CFILES= raidlib.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	${DRIVER}

clean:
	-rm -f *.o *.NEW *~ *Chunk*.bin
	-rm -f ${DRIVER} ${DERIVED} ${GARBAGE}

stripetest:	${OBJS} stripetest.o
	$(CC) -pg $(LDFLAGS) $(CFLAGS) -o $@ $(OBJS) stripetest.o $(LIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
