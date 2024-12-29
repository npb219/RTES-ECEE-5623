INCLUDE_DIRS = 
LIB_DIRS = 

CC=gcc

CDEFS=
CFLAGS= -O0 -g $(INCLUDE_DIRS) $(CDEFS)
//CFLAGS= -O3 -Wall -pg -msse3 -malign-double -g $(INCLUDE_DIRS) $(CDEFS)
LIBS=

DRIVER=ecctest
ASSIGNMENT = assignment_3_3

HFILES= ecclib.h
CFILES= ecctest.c ecclib.c assignment_3_3.c

SRCS= ${HFILES} ${CFILES}
DRIVER_OBJS = ecctest.o ecclib.o
ASSIGNMENT_OBJS = assignment_3_3.o ecclib.o

all:	${DRIVER} ${ASSIGNMENT}

clean:
	-rm -f *.o *.NEW *~ gmon.out
	-rm -f ${DRIVER} ${ASSIGNMENT} ${DERIVED} ${GARBAGE}

${ASSIGNMENT}:	${ASSIGNMENT_OBJS}
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $(ASSIGNMENT_OBJS) $(LIBS)

${DRIVER}:	${DRIVER_OBJS}
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $(DRIVER_OBJS) $(LIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<
