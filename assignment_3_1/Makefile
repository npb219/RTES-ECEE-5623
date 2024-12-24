INCLUDE_DIRS = 
LIB_DIRS = 
CC=gcc

CDEFS=
CFLAGS= -O0 -g -Wcpp $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lrt -lpthread

HFILES= 
CFILES= capture.c assignment_3_1.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	capture assignment_3_1

clean:
	-rm -f *.o *.d capture assignment_3_1
	-rm -f frames/*

distclean:
	-rm -f *.o *.d

capture: ${OBJS}
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o $(LIBS)

assignment_3_1: ${OBJS}
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $@.o $(LIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
