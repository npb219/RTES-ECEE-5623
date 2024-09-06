    //threading, scheduling and syslog libs
    #define _GNU_SOURCE
    #include <pthread.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include <errno.h>
    #include <sys/time.h>
    #include <sys/types.h>
    #include <sched.h>
    #include <unistd.h>
    #include <syslog.h>

    //create 64 threads
    //8 cpus
    #define NUM_THREADS 64
    #define NUM_CPUS 8

    //thread parameters type
    typedef struct
    {
        int threadIdx; //thread index
    } threadParams_t;


    // POSIX thread declarations and scheduling attributes
    //
    pthread_t threads[NUM_THREADS];             //threads array
    pthread_t mainthread;                       //main thread
    pthread_t startthread;                      //starter thread
    threadParams_t threadParams[NUM_THREADS];   //thread params

    pthread_attr_t fifo_sched_attr;             //fifo thread attributes
    pthread_attr_t orig_sched_attr;             //start attr
    struct sched_param fifo_param;              //fifo params

    #define SCHED_POLICY SCHED_FIFO//set scheduling policy to SCHED_FIFO
    #define MAX_ITERATIONS (1000000)//max iterations

    //print scheduler type
    void print_scheduler(void)
    {
        int schedType = sched_getscheduler(getpid());

        switch(schedType)
        {
            case SCHED_FIFO:
                syslog(LOG_CRIT,
                    "Pthread policy is SCHED_FIFO\n");
                break;
            case SCHED_OTHER:
                syslog(LOG_CRIT,
                    "Pthread policy is SCHED_OTHER\n");
                break;
            case SCHED_RR:
                syslog(LOG_CRIT,
                    "Pthread policy is SCHED_RR\n");
                break;
            default:
                syslog(LOG_CRIT,
                    "Pthread policy is UNKNOWN\n");
        }
    }

    //set up scheduler thread params
    void set_scheduler(void)
    {
        int max_prio, scope, rc, cpuidx;
        cpu_set_t cpuset;

        syslog(LOG_CRIT,
            "INITIAL ");
        print_scheduler();

        //assign sched policy
        pthread_attr_init(&fifo_sched_attr);
        pthread_attr_setinheritsched(&fifo_sched_attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&fifo_sched_attr, SCHED_POLICY);

        //assign cpu affinity
        CPU_ZERO(&cpuset);
        cpuidx=(3);
        CPU_SET(cpuidx, &cpuset);
        pthread_attr_setaffinity_np(&fifo_sched_attr, sizeof(cpu_set_t), &cpuset);
        
        //assign priority
        max_prio=sched_get_priority_max(SCHED_POLICY);
        fifo_param.sched_priority=max_prio;    

        //set sched alg
        if((rc=sched_setscheduler(getpid(), SCHED_POLICY, &fifo_param)) < 0)
            perror("sched_setscheduler");

        //set param to attributes
        pthread_attr_setschedparam(&fifo_sched_attr, &fifo_param);

        syslog(LOG_CRIT,
            "ADJUSTED ");
        print_scheduler();
    }



    //sum 0- thread index
    //\param threadp thread object
    void *counterThread(void *threadp)
    {
        int sum=0, i, rc, iterations;
        //local thread params
        threadParams_t *threadParams = (threadParams_t *)threadp;
        pthread_t mythread;
        double start=0.0, stop=0.0;
        struct timeval startTime, stopTime;

        //get time of day
        gettimeofday(&startTime, 0);
        //start time
        start = ((startTime.tv_sec * 1000000.0) + startTime.tv_usec)/1000000.0;

        //sum 0 -  thread idx + 1
        for(iterations=0; iterations < MAX_ITERATIONS; iterations++)
        {
            sum=0;
            for(i=1; i < (threadParams->threadIdx)+1; i++)
                sum=sum+i;
        }

        //stop time
        gettimeofday(&stopTime, 0);
        stop = ((stopTime.tv_sec * 1000000.0) + stopTime.tv_usec)/1000000.0;


        syslog(LOG_CRIT,
            "Thread idx=%d, sum[0...%d]=%d, running on CPU=%d, start=%lf, stop=%lf", 
            threadParams->threadIdx,
            threadParams->threadIdx, sum, sched_getcpu(),
            start, stop);
    }

    //create child threads
    //\param threadp thread object
    void *starterThread(void *threadp)
    {
        int i, rc;

        syslog(LOG_CRIT,
            "starter thread running on CPU=%d\n", sched_getcpu());

        //create child threads to run counter threads with fifo_sched_attr
        for(i=0; i < NUM_THREADS; i++)
        {
            threadParams[i].threadIdx=i;

            pthread_create(&threads[i],   // pointer to thread descriptor
                            &fifo_sched_attr,     // use FIFO RT max priority attributes
                            counterThread, // thread function entry point
                            (void *)&(threadParams[i]) // parameters to pass in
                            );

        }

        //wait for threads to finish
        for(i=0;i<NUM_THREADS;i++)
            pthread_join(threads[i], NULL);

    }


    int main (int argc, char *argv[])
    {
        int rc;
        int i, j;
        cpu_set_t cpuset;


        //start syslog
        openlog ("[COURSE:1][ASSIGNMENT:3]", LOG_NDELAY, LOG_DAEMON); 
        syslog(LOG_CRIT, argv[1]);


        //create scheduler and attributes
        set_scheduler();
        //remove all cpi from cpuset
        CPU_ZERO(&cpuset);

        // get affinity set for main thread
        mainthread = pthread_self();

        // Check the affinity mask assigned to the thread 
        rc = pthread_getaffinity_np(mainthread, sizeof(cpu_set_t), &cpuset);
        if (rc != 0)
            perror("pthread_getaffinity_np");
        else
        {
            syslog(LOG_CRIT,
                "main thread running on CPU=%d, CPUs =", sched_getcpu());

            for (j = 0; j < CPU_SETSIZE; j++)
                if (CPU_ISSET(j, &cpuset))
                    syslog(LOG_CRIT,
                        " %d", j);

            printf("\n");
        }
        //start starter thread
        pthread_create(&startthread,   // pointer to thread descriptor
                        &fifo_sched_attr,     // use FIFO RT max priority attributes
                        starterThread, // thread function entry point
                        (void *)0 // parameters to pass in
                        );
        //wait for starter thread to complete
        pthread_join(startthread, NULL);
        //done
        syslog(LOG_CRIT,
            "TEST COMPLETE\n");
    }
