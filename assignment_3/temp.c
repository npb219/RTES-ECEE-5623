#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <sched.h>
#include <syslog.h>

#define COUNT  128

typedef struct
{
    int threadIdx;
} threadParams_t;


// POSIX thread declarations and scheduling attributes
//
pthread_t threads[2];
threadParams_t threadParams[2];


// Unsafe global
int gsum=0;

void *incThread(void *threadp)
{
    int i;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    for(i=0; i<COUNT; i++)
    {
        gsum=gsum+i;
        //printf("Increment thread idx=%d, gsum=%d\n", threadParams->threadIdx, gsum);
        syslog(LOG_CRIT,
            "Thread idx=%d, sum[0...%d]=%d Running on core :%d\n", 
            threadParams->threadIdx + 1,
            threadParams->threadIdx,
            gsum,
            sched_getcpu());
    }
}


void *decThread(void *threadp)
{
    int i;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    for(i=0; i<COUNT; i++)
    {
        gsum=gsum-i;
        //printf("Decrement thread idx=%d, gsum=%d\n", threadParams->threadIdx, gsum);
        syslog(LOG_CRIT,
            "Thread idx=%d, sum[0...%d]=%d Running on core :%d\n", 
            threadParams->threadIdx + 1,
            threadParams->threadIdx,
            gsum,
            sched_getcpu());
    }
}




int main (int argc, char *argv[])
{
   int rc;
   int i=0;

   openlog ("[COURSE:1][ASSIGNMENT:3]", LOG_NDELAY, LOG_DAEMON); 
   syslog(LOG_CRIT, argv[1]);

   threadParams[i].threadIdx=i;
   pthread_create(&threads[i],   // pointer to thread descriptor
                  (void *)0,     // use default attributes
                  incThread, // thread function entry point
                  (void *)&(threadParams[i]) // parameters to pass in
                 );
   i++;

   threadParams[i].threadIdx=i;
   pthread_create(&threads[i], (void *)0, decThread, (void *)&(threadParams[i]));

   for(i=0; i<2; i++)
     pthread_join(threads[i], NULL);

   syslog(LOG_CRIT,
            "TEST COMPLETE\n");
}
