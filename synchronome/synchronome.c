// Noah Bowden, March 2025
//
// Synchronome Project
//
// Sequencer - 100 Hz  [gives semaphores to all other services]
//
//////////////////// For 1fps ////////////////////
// Service_1 - 5    Hz, every 20th Sequencer loop
// Service_2 - 5    Hz, every 20th Sequencer loop
// Service_3 - 1    Hz ,every 100th Sequencer loop
// Service_4 - 1    Hz ,every 100th Sequencer loop
//
// With the above, priorities by RM policy would be:
//
// Sequencer = RT_MAX	@ 100   Hz
// Servcie_1 = RT_MAX-1	@ 5     Hz
// Service_2 = RT_MAX-1	@ 5     Hz
// Service_3 = RT_MAX-2	@ 1     Hz
// Service_3 = RT_MAX-1	@ 1     Hz
//
//////////////////// For 10fps ////////////////////
// Service_1 - 20   Hz, every 5th Sequencer loop
// Service_2 - 20   Hz, every 5th Sequencer loop
// Service_3 - 10   Hz ,every 10th Sequencer loop
// Service_3 - 10   Hz ,every 10th Sequencer loop
//
// With the above, priorities by RM policy would be:
//
// Sequencer = RT_MAX	@ 100   Hz
// Servcie_1 = RT_MAX-1	@ 20    Hz
// Service_2 = RT_MAX-1	@ 20    Hz
// Service_3 = RT_MAX-2	@ 10    Hz
// Service_3 = RT_MAX-1	@ 10    Hz

// This is necessary for CPU affinity macros in Linux
#define _GNU_SOURCE

#include "capturelib.h"
#include <getopt.h>             /* getopt_long() */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>

#include <syslog.h>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <errno.h>
#include <sys/utsname.h>

#include <signal.h>

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_MSEC (1000000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (4)
#define TRUE (1)
#define FALSE (0)

#define NUM_THREADS (4)

// Of the available user space clocks, CLOCK_MONONTONIC_RAW is typically most precise and not subject to 
// updates from external timer adjustments
//
// However, some POSIX functions like clock_nanosleep can only use adjusted CLOCK_MONOTONIC or CLOCK_REALTIME

#define MY_CLOCK_TYPE CLOCK_MONOTONIC_RAW

//task aborts
int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE, abortS4=FALSE;
//task semephores
sem_t semS1, semS2, semS3, semS4;
//timer vals
struct timespec start_time_val;
double start_realtime;
unsigned long long sequencePeriods;

static timer_t timer_1;
static struct itimerspec itime = {{1,0}, {1,0}};
static struct itimerspec last_itime;

static unsigned long long seqCnt=0;

//thread params
typedef struct
{
    int threadIdx;//thread index
} threadParams_t;


void Sequencer(int id);//sequencer function

void *Service_1(void *threadp);//get frames and grey scale
void *Service_2(void *threadp);//diff detect
void *Service_3(void *threadp);//post process
void *Service_4(void *threadp);//save frames

//get time in msec
double getTimeMsec(void);
//get time in timespec
double realtime(struct timespec *tsptr);

//frame rate
int speed = 1;
//frame count desired
int counts_desired = 180;


// For background on high resolution time-stamps and clocks:
//
// 1) https://www.kernel.org/doc/html/latest/core-api/timekeeping.html
// 2) https://blog.regehr.org/archives/794 - Raspberry Pi
// 3) https://blog.trailofbits.com/2019/10/03/tsc-frequency-for-all-better-profiling-and-benchmarking/
// 4) http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/example-1/perfmon.c
// 5) https://blog.remibergsma.com/2013/05/12/how-accurately-can-the-raspberry-pi-keep-time/
//
// The Raspberry Pi does not ship with a TSC nor HPET counter to use as clocksource. Instead it relies on
// the STC that Raspbian presents as a clocksource. Based on the source code, “STC: a free running counter
// that increments at the rate of 1MHz”. This means it increments every microsecond.
//
// "sudo apt-get install adjtimex" for an interesting utility to adjust your system clock
//
//

// only works for x64 with proper privileges - has worked in past, but new security measures may
// prevent use
static inline unsigned long long tsc_read(void)
{
    unsigned int lo, hi;

    // RDTSC copies contents of 64-bit TSC into EDX:EAX
    asm volatile("rdtsc" : "=a" (lo), "=d" (hi));
    return (unsigned long long)hi << 32 | lo;
}

// not able to read unless enabled by kernel module
static inline unsigned ccnt_read (void)
{
    unsigned cc;
    asm volatile ("mrc p15, 0, %0, c15, c12, 1" : "=r" (cc));
    return cc;
}

// Function to display help
void print_usage() {
    printf("Usage: ./synchronome [-s <speed>] [-c <count>]\n");
    printf("   -s <speed>  Set capture speed (in 1 or 10 Hz)\n");
    printf("   -c <count>  Set capture qty\n");
}

void main(int argc, char *argv[])
{
    //time vals
    struct timespec current_time_val, current_time_res;
    double current_realtime, current_realtime_res;

    //log
    struct utsname sys_info;

    int i, rc, scope, flags=0;
    //cpu params
    cpu_set_t threadcpu;
    cpu_set_t allcpuset;

    pthread_t threads[NUM_THREADS];
    threadParams_t threadParams[NUM_THREADS];
    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio, cpuidx;

    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;

    pthread_attr_t main_attr;
    pid_t mainpid;

    int opt;

    // Parse command-line arguments
    while ((opt = getopt(argc, argv, "s:c:l:d:")) != -1) {
        switch (opt) {
            case 's':  // set speed
                speed = atoi(optarg);  // Convert argument to integer
                if (speed != 1 && speed != 10) {
                    fprintf(stderr, "Error: Speed must be 1 (default) or 10hz.\n");
                    print_usage();
                    exit(EXIT_FAILURE);
                }
                break;
            case 'c':  // set num captures
                counts_desired = atoi(optarg);  // Convert argument to integer
                
                break;
            case 'd': // save diff image
                en_diff_img = ( atoi(optarg) != 0 );
                break;
            case 'l': // enable laplace
                en_laplace = ( atoi(optarg) != 0 );
                break;
            default:
                print_usage();
                exit(EXIT_FAILURE);
        }
    }

    // Print the capture speed for verification
    printf("Capture speed set to %d Hz\n", speed);
    printf("Capture count set to %d Hz\n", counts_desired);
    if( en_laplace )
        printf("Enable laplace xfrm!");

    speed_hz = speed;

    //start syslog
    openlog ("[COURSE:4][Final Project]", LOG_NDELAY, LOG_DAEMON); 
    //syslog(LOG_CRIT, argv[1]);
    // Run uname -a and capture the output
    char buffer[256];
    FILE *fp = popen("uname -a", "r");
    if (fp) {
        if (fgets(buffer, sizeof(buffer), fp) != NULL) {
            syslog(LOG_CRIT, "%s", buffer);
        }
        pclose(fp);
    }

    //init camera
    init();

   CPU_ZERO(&allcpuset);
    //get cpu avail
   for(i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);


    // initialize the sequencer semaphores
    //
    if (sem_init (&semS1, 0, 0)) { syslog(LOG_CRIT, "Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { syslog(LOG_CRIT, "Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { syslog(LOG_CRIT, "Failed to initialize S3 semaphore\n"); exit (-1); }
    if (sem_init (&semS4, 0, 0)) { syslog(LOG_CRIT, "Failed to initialize S4 semaphore\n"); exit (-1); }

    mainpid=getpid();
    //set priorities
    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);
    //set up sched fifo
    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");

    //get pthread scope
    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      syslog(LOG_CRIT, "PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      syslog(LOG_CRIT, "PTHREAD SCOPE PROCESS\n");
    else
      syslog(LOG_CRIT, "PTHREAD SCOPE UNKNOWN\n");

    syslog(LOG_CRIT, "rt_max_prio=%d\n", rt_max_prio);
    syslog(LOG_CRIT, "rt_min_prio=%d\n", rt_min_prio);

    //create threads' attr: fifo, explicit sched, cpu 3
    for(i=0; i < NUM_THREADS; i++)
    {
        //set thread 1 to cpu 1, thread 2 and 3 to cpu 2, thrad 4 to cpu 3
        CPU_ZERO(&threadcpu);
        if( i == 0 )
            cpuidx=(1);
        else if( i == 1 || i == 2 )
            cpuidx=(2);
        else
            cpuidx=(3);
        CPU_SET(cpuidx, &threadcpu);

        //explicit, sched fifo
        rc=pthread_attr_init(&rt_sched_attr[i]);
        rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
        rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
        rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

        //give thread 3 lower priority
        //all others max (threads 1 and 4 on their own thread)
        if( i == 2 )
            rt_param[i].sched_priority=rt_max_prio-2;
        else
            rt_param[i].sched_priority=rt_max_prio-1;
        
        pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

        threadParams[i].threadIdx=i;
    }
   
    syslog(LOG_CRIT, "Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

    // Create Service threads which will block awaiting release for:
    //

    // Servcie_1
    //
    rt_param[0].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    rc=pthread_create(&threads[0],               // pointer to thread descriptor
                      &rt_sched_attr[0],         // use specific attributes
                      //(void *)0,               // default attributes
                      Service_1,                 // thread function entry point
                      (void *)&(threadParams[0]) // parameters to pass in
                     );
    if(rc < 0)
        perror("pthread_create for service 1");
    else
        syslog(LOG_CRIT, "pthread_create successful for service 1\n");


    // Service_2
    //
    rt_param[1].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    rc=pthread_create(&threads[1], &rt_sched_attr[1], Service_2, (void *)&(threadParams[1]));
    if(rc < 0)
        perror("pthread_create for service 2");
    else
        syslog(LOG_CRIT, "pthread_create successful for service 2\n");


    // Service_3
    //
    rt_param[2].sched_priority=rt_max_prio-2;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    rc=pthread_create(&threads[2], &rt_sched_attr[2], Service_3, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 3");
    else
        syslog(LOG_CRIT, "pthread_create successful for service 3\n");

    // Service_4
    //
    rt_param[3].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
    rc=pthread_create(&threads[3], &rt_sched_attr[3], Service_4, (void *)&(threadParams[3]));
    if(rc < 0)
        perror("pthread_create for service 3");
    else
        syslog(LOG_CRIT, "pthread_create successful for service 3\n");


 
    // Create Sequencer thread, which like a cyclic executive, is highest prio
    syslog(LOG_CRIT, "Start sequencer\n");
    sequencePeriods=60;

    // Sequencer = RT_MAX	@ 100 Hz
    //
    /* set up to signal SIGALRM if timer expires */
    timer_create(CLOCK_REALTIME, NULL, &timer_1);

    signal(SIGALRM, (void(*)()) Sequencer);


    /* arm the interval timer */
    itime.it_interval.tv_sec = 0;
    itime.it_interval.tv_nsec = 10000000;
    itime.it_value.tv_sec = 0;
    itime.it_value.tv_nsec = 10000000;

    //start timer
    timer_settime(timer_1, flags, &itime, &last_itime);

    //set start time
    clock_gettime(MY_CLOCK_TYPE, &start_time_val); start_realtime=realtime(&start_time_val);


    for(i=0;i<NUM_THREADS;i++)
    {
        if(rc=pthread_join(threads[i], NULL) < 0)
		perror("main pthread_join");
    }

    //uninit camera
    uninit();


   syslog(LOG_CRIT, "TEST COMPLETE");
}



void Sequencer(int id)
{
    struct timespec current_time_val;
    double current_realtime;
    int rc, flags=0;

    // received interval timer signal
           
    seqCnt++;

    // Release each service at a sub-rate of the generic sequencer rate

    if( speed == 1 ) //1hz
    {
        // Servcie_1 = RT_MAX-1	@ 5 Hz
        if((seqCnt % 20) == 1) sem_post(&semS1);
        // Servcie_2 = RT_MAX-1	@ 5 Hz
        if((seqCnt % 20) == 1) sem_post(&semS2);
        // Servcie_3 = RT_MAX-2	@ 1 Hz
        if((seqCnt % 100) == 1 ) sem_post(&semS3);
        // Servcie_4 = RT_MAX-1	@ 1 Hz
        if(((seqCnt) % 100) == 1 ) sem_post(&semS4);
    }
    else //10hz
    {

        // Servcie_1 = RT_MAX-1	@ 20 Hz
        if((seqCnt % 5) == 1) sem_post(&semS1);
        // Servcie_2 = RT_MAX-1	@ 20 Hz
        if((seqCnt % 5) == 1) sem_post(&semS2);
        // Servcie_3 = RT_MAX-2	@ 10 Hz
        //if((seqCnt % 10) == 1 ) sem_post(&semS3);
        if((seqCnt % 10) == 1 ) sem_post(&semS3);
        // Servcie_4 = RT_MAX-1	@ 10 Hz
        //if(((seqCnt) % 10) == 1 ) sem_post(&semS4);
        if((seqCnt % 10) == 1 ) sem_post(&semS4);
    }


    
    if(abortTest)
    {
        // disable interval timer
        itime.it_interval.tv_sec = 0;
        itime.it_interval.tv_nsec = 0;
        itime.it_value.tv_sec = 0;
        itime.it_value.tv_nsec = 0;
        timer_settime(timer_1, flags, &itime, &last_itime);

	// shutdown all services
        sem_post(&semS1); sem_post(&semS2); sem_post(&semS3); sem_post(&semS4);

        abortS1=TRUE; abortS2=TRUE; abortS3=TRUE; abortS4=TRUE;
    }

}



void *Service_1(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    double startread_realtime;
    double stopread_realtime;
    double diff_realtime;
    double start_proc_realtime;
    unsigned long long S1Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);

    while(!abortS1) // check for synchronous abort request
    {
	    // wait for service request from the sequencer, a signal handler or ISR in kernel
        sem_wait(&semS1);

        S1Cnt++;

        // on order of up to milliseconds of latency to get time
        // clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); startread_realtime = realtime(&current_time_val) - start_realtime;

	    // capture frame
        capture();
        
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); stopread_realtime = realtime(&current_time_val) - start_realtime;
	    
        // syslog(LOG_CRIT, "Thread 1 start %d @ <%6.9lf> on core <%d>", threadParams->threadIdx, current_realtime-start_realtime, sched_getcpu());
        //syslog(LOG_CRIT, "Thread 1 start @ <%6.9lf> duration: <%6.9lf>", startread_realtime, stopread_realtime-startread_realtime);
    }

    // Resource shutdown here
    //
    pthread_exit((void *)0);
}


void *Service_2(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    double startread_realtime;
    double stopread_realtime;
    unsigned long long S2Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);

    while(!abortS2)
    {
        sem_wait(&semS2);
        S2Cnt++;

clock_gettime(MY_CLOCK_TYPE, &current_time_val); startread_realtime = realtime(&current_time_val) - start_realtime;

        //diff image
        performDiff();

clock_gettime(MY_CLOCK_TYPE, &current_time_val); stopread_realtime = realtime(&current_time_val) - start_realtime;

        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        // syslog(LOG_CRIT, "Thread 2 start %d @ <%6.9lf> on core <%d>", threadParams->threadIdx, current_realtime-start_realtime, sched_getcpu());
        //syslog(LOG_CRIT, "Thread 2 start @ <%6.9lf> duration: <%6.9lf>", startread_realtime, stopread_realtime-startread_realtime);
    }

    pthread_exit((void *)0);
}


void *Service_3(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    double startread_realtime;
    double stopread_realtime;
    unsigned long long S3Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    double start_proc_realtime;

    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);

    while(!abortS3)
    {
        sem_wait(&semS3);
        S3Cnt++;

clock_gettime(MY_CLOCK_TYPE, &current_time_val); startread_realtime = realtime(&current_time_val) - start_realtime;
        
        //post process image
        postProcess();

clock_gettime(MY_CLOCK_TYPE, &current_time_val); stopread_realtime = realtime(&current_time_val) - start_realtime;

        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        // syslog(LOG_CRIT, "Thread 3 start %d @ <%6.9lf> on core <%d>", threadParams->threadIdx, current_realtime-start_realtime, sched_getcpu());
        //syslog(LOG_CRIT, "Thread 3 start @ <%6.9lf> duration: <%6.9lf>", startread_realtime, stopread_realtime-startread_realtime);

    }

    pthread_exit((void *)0);
}

void *Service_4(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    double startread_realtime;
    double stopread_realtime;
    double start_proc_realtime;
    unsigned long long S4Cnt=0;
    threadParams_t *threadParams = (threadParams_t *)threadp;

    // Start up processing and resource initialization
    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);

    while(!abortS4) // check for synchronous abort request
    {
	    // wait for service request from the sequencer, a signal handler or ISR in kernel
        sem_wait(&semS4);

        S4Cnt++;

clock_gettime(MY_CLOCK_TYPE, &current_time_val); startread_realtime = realtime(&current_time_val) - start_realtime;

	    // save image
        if( saveImg() == counts_desired )
            abortTest = 1;

clock_gettime(MY_CLOCK_TYPE, &current_time_val); stopread_realtime = realtime(&current_time_val) - start_realtime;

	    // on order of up to milliseconds of latency to get time
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        // syslog(LOG_CRIT, "Thread 4 start %d @ <%6.9lf> on core <%d>", threadParams->threadIdx, current_realtime-start_realtime, sched_getcpu());
        //syslog(LOG_CRIT, "Thread 4 start @ <%6.9lf> duration: <%6.9lf>", startread_realtime, stopread_realtime-startread_realtime);
    }

    // Resource shutdown here
    //
    pthread_exit((void *)0);
}

//get ttime in mSec
double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(MY_CLOCK_TYPE, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

//get real time
double realtime(struct timespec *tsptr)
{
    return ((double)(tsptr->tv_sec) + (((double)tsptr->tv_nsec)/1000000000.0));
}

