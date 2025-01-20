#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <syslog.h>

#include "raidlib.h"

#define THREADED

//typedefs
typedef double FLOAT;
typedef unsigned int UINT32;
typedef unsigned long long int UINT64;
typedef unsigned char UINT8;




int main(int argc, char *argv[])
{
    int bytesWritten, bytesRestored;
    char rc;

    //test timekeeping
    UINT64 microsecs=0, millisecs=0;
    unsigned int thread_idx;
    FLOAT temp, fnow, fstart;
    struct timespec now, start;

    // For testing, if no data is lost (erased), then the zero default
    // indicates that no data chunk was lost.
    int chunkToRebuild=0;

    if(argc < 3)
    {
        printf("useage: stripetest inputfile outputfile <sector to restore>\n");
        exit(-1);
    }
    
    if(argc >= 4)
    {
	    sscanf(argv[3], "%d", &chunkToRebuild);
        printf("chunk to restore = %d\n", chunkToRebuild);
        traceOn();
        write_trace("chunk to restore = %d\n", chunkToRebuild);
    }

    clock_gettime(CLOCK_MONOTONIC, &start);
    fstart = (FLOAT)start.tv_sec + (FLOAT)start.tv_nsec / 1000000000.0;
    clock_gettime(CLOCK_MONOTONIC, &now);
    fnow = (FLOAT)now.tv_sec + (FLOAT)now.tv_nsec / 1000000000.0;
    printf("start striping file at %lf\n", fnow - fstart);
  
    // What is the meaning of the "0" argument here? 
    // This is an offset in sectors that is not actually used at all in raidlib.c, so we might
    // want to depricate this argument.
    #ifdef THREADED
    //bytesWritten=stripeFileThread("Cactus-12mpixel.ppm", 0); 
    bytesWritten=stripeFileThread(argv[1], 0); 
    #else
    bytesWritten=stripeFile(argv[1], 0); 
    #endif


    temp = fnow;
    clock_gettime(CLOCK_MONOTONIC, &now);
    fnow = (FLOAT)now.tv_sec + (FLOAT)now.tv_nsec / 1000000000.0;
    printf("end striping file at %lf\n", fnow - fstart);
    printf("time elapsed: %lf\n", fnow - temp);

    write_trace("%s input file was written as 4 data chunks + 1 XOR parity on 5 devices 1...5\n", argv[1]);
    printf("%s input file was written as 4 data chunks + 1 XOR parity on 5 devices 1...5\n", argv[1]);

    write_trace("Enter chunk you have erased or 0 for none:");
    printf("Enter chunk you have erased or 0 for none:");
    fscanf(stdin, "%d", &chunkToRebuild);
    write_trace("Got %d\n", chunkToRebuild);
    printf("Got %d\n", chunkToRebuild);

    clock_gettime(CLOCK_MONOTONIC, &now);
    fnow = (FLOAT)now.tv_sec + (FLOAT)now.tv_nsec / 1000000000.0;
    printf("start rebuild and save at %lf\n", fnow - fstart);

    if(chunkToRebuild > 0)
    {
        write_trace("Will rebuild StripeChunk%1d.bin\n", chunkToRebuild);
        write_trace("working on restoring file ...\n");
        printf("Will rebuild StripeChunk%1d.bin\n", chunkToRebuild);
        printf("working on restoring file ...\n");

        // What is the meaning of the "0" argument here? 
        bytesRestored=restoreFile(argv[2], 0, bytesWritten, chunkToRebuild); 
    }
    else
    {
        write_trace("Nothing erased, so nothing to restore\n");
        printf("Nothing erased, so nothing to restore\n");
        // What is the meaning of the first "0" argument and the second zero here? 
        bytesRestored=restoreFile(argv[2], 0, bytesWritten, 0); 
    }

    temp = fnow;
    clock_gettime(CLOCK_MONOTONIC, &now);
    fnow = (FLOAT)now.tv_sec + (FLOAT)now.tv_nsec / 1000000000.0;
    printf("end writing file at %lf\n", fnow - fstart);
    printf("time elapsed: %lf\n", fnow - temp);

    write_trace("FINISHED\n");
    printf("FINISHED\n");
}
