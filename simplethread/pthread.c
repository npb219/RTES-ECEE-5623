    //assignment 1-2

    //include threading, stdio and syslog libs
    #include <pthread.h>
    #include <stdlib.h>
    #include <stdio.h>
    #include <sched.h>
    #include <syslog.h>

    //create 128 threads
    #define NUM_THREADS 128//12

    //thread parameters type
    typedef struct
    {
        int threadIdx; //thread index
    } threadParams_t;


    // POSIX thread declarations and scheduling attributes
    //
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];

    // sums 1 - thread index of threadp
    // \param threadp thread object
    void *counterThread(void *threadp)
    {
        int sum=1, i;

        //local thread param
        threadParams_t *threadParams = (threadParams_t *)threadp;

        //sum 1 - thread indx + 1
        for(i=1; i < (threadParams->threadIdx)+1; i++)
            sum=sum+i;
        
        //print to syslog
        syslog(LOG_CRIT,
                "Thread idx=%d, sum[0...%d]=%d\n", 
                threadParams->threadIdx + 1,
                threadParams->threadIdx, sum);

        
    }


    int main (int argc, char *argv[])
    {
    int rc;
    int i;

    //start syslog
    openlog ("[COURSE:1][ASSIGNMENT:2]", LOG_NDELAY, LOG_DAEMON); 
    syslog(LOG_CRIT, argv[1]);

    //create threads
    for(i=0; i < NUM_THREADS; i++)
    {
        threadParams[i].threadIdx=i;

        pthread_create(&threads[i],   // pointer to thread descriptor
                        (void *)0,     // use default attributes
                        counterThread, // thread function entry point
                        (void *)&(threadParams[i]) // parameters to pass in
                        );

    }

    //wait for threads to finish
    for(i=0;i<NUM_THREADS;i++)
        pthread_join(threads[i], NULL);

    //done
    syslog(LOG_CRIT,
                "TEST COMPLETE\n");
    }
