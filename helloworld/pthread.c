    //assignment 1-1

    //include stdio, threading and scheduling libs, and syslog
    #include <pthread.h>
    #include <stdlib.h>
    #include <stdio.h>
    #include <sched.h>
    #include <syslog.h>

    //create 1 thread
    #define NUM_THREADS 1

    //struct for thread parameters
    typedef struct
    {
        int threadIdx;  // Detailed description after the member
    } threadParams_t;


    // POSIX thread declarations and scheduling attributes
    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];

    //prints hello from the thread
    // \param threadp thread object
    void *hello_thread(void *threadp)
    {
        syslog(LOG_CRIT,
                "Hello World from Thread!\n");
    }


    int main (int argc, char *argv[])
    {

    //start syslogs for autograder
    openlog ("[COURSE:1][ASSIGNMENT:1]", LOG_NDELAY, LOG_DAEMON); 
    syslog(LOG_CRIT, argv[1]);

    //print hello world from main thread
    syslog(LOG_CRIT, "Hello World from Main!\n");

    //assign thread index to thread
    threadParams[0].threadIdx=1;

    //create thread
    pthread_create(&threads[0],   // pointer to thread descriptor
                    (void *)0,     // use default attributes
                    hello_thread, // thread function entry point
                    (void *)&(threadParams[0]) // parameters to pass in
                    );
    //wait for thread to finish
    pthread_join(threads[0], NULL);
    }
