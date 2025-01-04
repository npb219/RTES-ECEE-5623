#define _GNU_SOURCE
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <assert.h>

#include <stdarg.h>  // For va_list, va_start, va_end

#include "raidlib.h"

#ifdef RAID64
#include "raidlib64.h"
#define PTR_CAST (unsigned long long *)
#else
#include "raidlib.h"
#define PTR_CAST (unsigned char *)
#endif

#include <pthread.h>
#include <sched.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/sysinfo.h>
//helper threads
#define num_threads 3
#define NUM_STRIPES 128

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_MSEC (1000000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (4)
#define TRUE (1)
#define FALSE (0)
#define NUM_THREADS (3)
#define MY_CLOCK_TYPE CLOCK_MONOTONIC_RAW
int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE;
//task semephores
sem_t semS1, semS2, semS3;
static timer_t timer_1;
static struct itimerspec itime = {{1,0}, {1,0}};
static struct itimerspec last_itime;
int seqCnt = 0;
void Sequencer(int id);

//strypes struct
typedef struct {
    unsigned char LBA1[SECTOR_SIZE];
    unsigned char LBA2[SECTOR_SIZE];
    unsigned char LBA3[SECTOR_SIZE];
    unsigned char LBA4[SECTOR_SIZE];
    unsigned char PLBA[SECTOR_SIZE];
} stripe_t;

// Ring buffer struct
typedef struct {
    stripe_t buffer[NUM_STRIPES];
    int read_idx;  // reader thread idx
    int write_idx; // writer thread idx
    int count;     // num items in buffer
    pthread_mutex_t mutex;
    pthread_cond_t cond;
} ring_buffer_t;

pthread_t threads[num_threads];
int readDone = 0;
int xorDone = 0;

//thread args
typedef struct _threadArgs
{
    int thread_idx;
    int offsetSectors;
    char *inputFileName;
    ring_buffer_t *r_buffer;
    ring_buffer_t *w_buffer;
} threadArgsType;


pthread_attr_t fifo_sched_attr;
pthread_attr_t orig_sched_attr;
struct sched_param fifo_param;


static int printTrace=0;
static FILE *tracefile;

// RAID-5 encoding
//
// This is 80% capacity with 1/5 LBAs used for parity.
//
// Only handles single faults.
//
// PRECONDITIONS:
// 1) LBA pointeres must have memory allocated for them externally
// 2) Blocks pointer to by LBAs are initialized with data
//
// POST-CONDITIONS:
// 1) Contents of PLBA is modified and contains the computed parity using XOR
//
void xorLBA(unsigned char *LBA1,
	    unsigned char *LBA2,
	    unsigned char *LBA3,
	    unsigned char *LBA4,
	    unsigned char *PLBA)
{
    int idx;
    for(idx=0; idx<SECTOR_SIZE; idx++)
        *(PLBA+idx)=(*(LBA1+idx))^(*(LBA2+idx))^(*(LBA3+idx))^(*(LBA4+idx));
        //PLBA[idx]=LBA1[idx]^LBA2[idx]^LBA3[idx]^LBA4[idx];
}


// RAID-5 Rebuild
//
// Provide any 3 of the original LBAs and the Parity LBA to rebuild the RLBA
//
// If the Parity LBA was lost, then it can be rebuilt by simply re-encoding.
// 
void rebuildLBA(unsigned char *LBA1,
	        unsigned char *LBA2,
	        unsigned char *LBA3,
	        unsigned char *PLBA,
	        unsigned char *RLBA)
{
    int idx;
    unsigned char checkParity;

    for(idx=0; idx<SECTOR_SIZE; idx++)
    {
        // Parity check word is simply XOR of remaining good LBAs
        checkParity=(*(LBA1+idx))^(*(LBA2+idx))^(*(LBA3+idx));

        // Rebuilt LBA is simply XOR of original parity and parity check word
        // which will preserve original parity computed over the 4 LBAs
        *(RLBA+idx) =(*(PLBA+idx))^(checkParity);
    }
}


int checkEquivLBA(unsigned char *LBA1,
		  unsigned char *LBA2)
{
    int idx;

    for(idx=0; idx<SECTOR_SIZE; idx++)
    {
        if((*(LBA1+idx)) != (*(LBA2+idx)))
	    {
            write_trace("EQUIV CHECK MISMATCH @ byte %d: LBA1=0x%x, LBA2=0x%x\n", idx, (*LBA1+idx), (*LBA2+idx));
	        return ERROR;
	    }
    }

    return OK;
}


// returns bytes written or ERROR code
// 
int stripeFile(char *inputFileName, int offsetSectors)
{
    int fd[5], idx;
    FILE *fdin;
    unsigned char stripe[5*512];
    int offset=0, bread=0, btoread=(4*512), bwritten=0, btowrite=(512), sectorCnt=0, byteCnt=0;

    fdin = fopen(inputFileName, "r");
    fd[0] = open("StripeChunk1.bin", O_RDWR | O_CREAT, 00644);
    fd[1] = open("StripeChunk2.bin", O_RDWR | O_CREAT, 00644);
    fd[2] = open("StripeChunk3.bin", O_RDWR | O_CREAT, 00644);
    fd[3] = open("StripeChunk4.bin", O_RDWR | O_CREAT, 00644);
    fd[4] = open("StripeChunkXOR.bin", O_RDWR | O_CREAT, 00644);


    do
    {

        // read a stripe or to end of file
        offset=0, bread=0, btoread=(4*512);
        do
        {
            bread=fread(&stripe[offset], 1, btoread, fdin); 
            offset+=bread;
            btoread=(4*512)-bread;
        }
        while (!(feof(fdin)) && (btoread > 0));


        if((offset < (4*512)) && (feof(fdin)))
        {
            write_trace("hit end of file\n");
            bzero(&stripe[offset], btoread);
            byteCnt+=offset;
        }
        else
        {
            write_trace("read full stripe\n");
            assert(offset == (4*512));
            byteCnt+=(4*512);
        };

        // computer xor code for stripe
        //
        xorLBA(PTR_CAST &stripe[0],
               PTR_CAST &stripe[512],
               PTR_CAST &stripe[1024],
               PTR_CAST &stripe[1536],
               PTR_CAST &stripe[2048]);


        // write out the stripe + xor code
        //
        offset=0, bwritten=0, btowrite=(512);
        do
        {
            bwritten=write(fd[0], &stripe[offset], 512); 
            offset+=bwritten;
            btowrite=(512)-bwritten;
        }
        while (btowrite > 0);

        offset=512, bwritten=0, btowrite=(512);
        do
        {
            bwritten=write(fd[1], &stripe[offset], 512); 
            offset+=bwritten;
            btowrite=(512)-bwritten;
        }
        while (btowrite > 0);

        offset=1024, bwritten=0, btowrite=(512);
        do
        {
            bwritten=write(fd[2], &stripe[offset], 512); 
            offset+=bwritten;
            btowrite=(512)-bwritten;
        }
        while (btowrite > 0);

        offset=1536, bwritten=0, btowrite=(512);
        do
        {
            bwritten=write(fd[3], &stripe[offset], 512); 
            offset+=bwritten;
            btowrite=(512)-bwritten;
        }
        while (btowrite > 0);

        offset=2048, bwritten=0, btowrite=(512);
        do
        {
            bwritten=write(fd[4], &stripe[offset], 512); 
            offset+=bwritten;
            btowrite=(512)-bwritten;
        }
        while (btowrite > 0);

        sectorCnt+=4;

    }
    while (!(feof(fdin)));

    fclose(fdin);
    for(idx=0; idx < 5; idx++) close(fd[idx]);

    return(byteCnt);
}


// returns bytes read or ERROR code
// 
// missingChunk = 0 for no missing
//              = 1 ... 4 for missing data chunk
//              = 5 for missing XOR chunk
//
// Ugly repeated code needs to be refactored, but works for now.
//
int restoreFile(char *outputFileName, int offsetSectors, int fileLength, int missingChunk)
{
    int fd[5], idx;
    FILE *fdout;
    unsigned char stripe[5*512];
    int offset=0, bread=0, btoread=(4*512), bwritten=0, btowrite=(512), sectorCnt=fileLength/512;
    int stripeCnt=fileLength/(4*512);
    int lastStripeBytes = fileLength % (4*512);

    fdout = fopen(outputFileName, "w");

    fd[0] = open("StripeChunk1.bin", O_RDWR | O_CREAT, 00644);
    fd[1] = open("StripeChunk2.bin", O_RDWR | O_CREAT, 00644);
    fd[2] = open("StripeChunk3.bin", O_RDWR | O_CREAT, 00644);
    fd[3] = open("StripeChunk4.bin", O_RDWR | O_CREAT, 00644);
    fd[4] = open("StripeChunkXOR.bin", O_RDWR | O_CREAT, 00644);


    for(idx=0; idx < stripeCnt; idx++)
    {
        // read in the stripe + xor code

        if(missingChunk == 1)
	{
		write_trace("will rebuild chunk 1\n");
	}
	else
	{
            offset=0, bread=0, btoread=(512);
            do
            {
                bread=read(fd[0], &stripe[offset], 512); 
                offset+=bread;
                btoread=(512)-bread;
            }
            while (btoread > 0);
	}


        if(missingChunk == 2)
	{
		write_trace("will rebuild chunk 2\n");
	}
	else
	{
            offset=512, bread=0, btoread=(512);
            do
            {
                bread=read(fd[1], &stripe[offset], 512); 
                offset+=bread;
                btoread=(512)-bread;
            }
            while (btoread > 0);
	}


        if(missingChunk == 3)
	{
		write_trace("will rebuild chunk 3\n");
	}
	else
	{
            offset=1024, bread=0, btoread=(512);
            do
            {
                bread=read(fd[2], &stripe[offset], 512); 
                offset+=bread;
                btoread=(512)-bread;
            }
            while (btoread > 0);
	}

        if(missingChunk == 4)
	{
		write_trace("will rebuild chunk 4\n");
	}
	else
	{
            offset=1536, bread=0, btoread=(512);
            do
            {
                bread=read(fd[3], &stripe[offset], 512); 
                offset+=bread;
                btoread=(512)-bread;
            }
            while (btoread > 0);
	}

        if(missingChunk == 5)
	{
		write_trace("will rebuild chunk 5\n");
	}
	else
	{
            offset=2048, bread=0, btoread=(512);
            do
            {
                bread=read(fd[4], &stripe[offset], 512); 
                offset+=bread;
                btoread=(512)-bread;
            }
            while (btoread > 0);
	}

        // OPTION - check XOR here or re-compute it
        //
	if(missingChunk == 1)
	{
		rebuildLBA( PTR_CAST &stripe[512], 
			    PTR_CAST &stripe[1024], 
			    PTR_CAST &stripe[1536], 
			    PTR_CAST &stripe[2048], 
			    PTR_CAST &stripe[0]);
	}

	if(missingChunk == 2)
	{
		rebuildLBA( PTR_CAST &stripe[0], 
		  	    PTR_CAST &stripe[1024], 
			    PTR_CAST &stripe[1536], 
			    PTR_CAST &stripe[2048], 
			    PTR_CAST &stripe[512]);
	}
	if(missingChunk == 3)
	{
		rebuildLBA(
			   PTR_CAST &stripe[0],
		           PTR_CAST &stripe[512],
		           PTR_CAST &stripe[1536],
		           PTR_CAST &stripe[2048],
		           PTR_CAST &stripe[1024]
			  );
	}
	if(missingChunk == 4)
	{
		rebuildLBA(
			   PTR_CAST &stripe[0],
		           PTR_CAST &stripe[512],
		           PTR_CAST &stripe[1024],
		           PTR_CAST &stripe[2048],
		           PTR_CAST &stripe[1536]
			  );
	}
	if(missingChunk == 5)
	{
		rebuildLBA(
			   PTR_CAST &stripe[0],
		           PTR_CAST &stripe[512],
		           PTR_CAST &stripe[1024],
		           PTR_CAST &stripe[1536],
		           PTR_CAST &stripe[2048]
			  );
	}
	

       // write a full stripe
        offset=0, bwritten=0, btowrite=(4*512);

        do
        {
            bwritten=fwrite(&stripe[offset], 1, btowrite, fdout); 
            offset+=bwritten;
            btowrite=(4*512)-bwritten;
        }
        while ((btowrite > 0));

    }


    if(lastStripeBytes)
    {
        // read in the parital stripe + xor code
        //
        if(missingChunk == 1)
	{
		write_trace("will rebuild chunk 1\n");
	}
	else
	{
            offset=0, bread=0, btoread=(512);
            do
            {
                bread=read(fd[0], &stripe[offset], 512); 
                offset+=bread;
                btoread=(512)-bread;
            }
            while (btoread > 0);
	}

        if(missingChunk == 2)
	{
		write_trace("will rebuild chunk 2\n");
	}
	else
	{
            offset=512, bread=0, btoread=(512);
            do
            {
                bread=read(fd[1], &stripe[offset], 512); 
                offset+=bread;
                btoread=(512)-bread;
            }
            while (btoread > 0);
	}

        if(missingChunk == 3)
	{
		write_trace("will rebuild chunk 3\n");
	}
	else
	{
            offset=1024, bread=0, btoread=(512);
            do
            {
                bread=read(fd[2], &stripe[offset], 512); 
                offset+=bread;
                btoread=(512)-bread;
            }
            while (btoread > 0);
	}

        if(missingChunk == 4)
	{
		write_trace("will rebuild chunk 4\n");
	}
	else
	{
            offset=1536, bread=0, btoread=(512);
            do
            {
                bread=read(fd[3], &stripe[offset], 512); 
                offset+=bread;
                btoread=(512)-bread;
            }
            while (btoread > 0);
	}


        if(missingChunk == 5)
	{
		write_trace("will rebuild chunk 5\n");
	}
	else
	{
            offset=2048, bread=0, btoread=(512);
            do
            {
                bread=read(fd[4], &stripe[offset], 512); 
                offset+=bread;
                btoread=(512)-bread;
            }
            while (btoread > 0);
	}

        // OPTION - check XOR here
        //
	if(missingChunk == 1)
	{
		rebuildLBA( PTR_CAST &stripe[512], 
			    PTR_CAST &stripe[1024], 
			    PTR_CAST &stripe[1536], 
			    PTR_CAST &stripe[2048], 
			    PTR_CAST &stripe[0]);
	}

	if(missingChunk == 2)
	{
		rebuildLBA( PTR_CAST &stripe[0], 
		  	    PTR_CAST &stripe[1024], 
			    PTR_CAST &stripe[1536], 
			    PTR_CAST &stripe[2048], 
			    PTR_CAST &stripe[512]);
	}
	if(missingChunk == 3)
	{
		rebuildLBA(
			   PTR_CAST &stripe[0],
		           PTR_CAST &stripe[512],
		           PTR_CAST &stripe[1536],
		           PTR_CAST &stripe[2048],
		           PTR_CAST &stripe[1024]
			  );
	}
	if(missingChunk == 4)
	{
		rebuildLBA(
			   PTR_CAST &stripe[0],
		           PTR_CAST &stripe[512],
		           PTR_CAST &stripe[1024],
		           PTR_CAST &stripe[2048],
		           PTR_CAST &stripe[1536]
			  );
	}
	if(missingChunk == 5)
	{
		rebuildLBA(
			   PTR_CAST &stripe[0],
		           PTR_CAST &stripe[512],
		           PTR_CAST &stripe[1024],
		           PTR_CAST &stripe[1536],
		           PTR_CAST &stripe[2048]
			  );
	}

       // write a partial stripe
        offset=0, bwritten=0, btowrite=(lastStripeBytes);

        do
        {
            bwritten=fwrite(&stripe[offset], 1, btowrite, fdout); 
            offset+=bwritten;
            btowrite=lastStripeBytes-bwritten;
        }
        while ((btowrite > 0));
    }


    fclose(fdout);
    for(idx=0; idx < 5; idx++) close(fd[idx]);

    return(fileLength);
}

//################################# threaded attempts #################################



// Initialize the ring buffer
void init_ring_buffer(ring_buffer_t *rb) {
    rb->read_idx = 0;
    rb->write_idx = 0;
    rb->count = 0;
    pthread_mutex_init(&rb->mutex, NULL);
    pthread_cond_init(&rb->cond, NULL);
}

int lock_with_timeout(pthread_mutex_t *mutex, long timeout_ms) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_ms / 1000;
    ts.tv_nsec += (timeout_ms % 1000) * 1000000;
    
    // Ensure the nanoseconds part doesn't overflow into seconds
    if (ts.tv_nsec >= 1000000000) {
        ts.tv_nsec -= 1000000000;
        ts.tv_sec += 1;
    }

    // Try locking the mutex with a timeout
    int result = pthread_mutex_timedlock(mutex, &ts);
    
    if (result == 0) {
        return 0; // Mutex locked successfully
    } else if (result == ETIMEDOUT) {
        return -1; // Timeout reached
    } else {
        return -2; // Some other error occurred
    }
}

// Add an item to the ring buffer
void add_to_buffer(ring_buffer_t *rb, stripe_t *stripe) {
    //pthread_mutex_lock(&rb->mutex);
    // Wait if the buffer is full
    while (rb->count == NUM_STRIPES) {
        //pthread_cond_wait(&rb->cond, &rb->mutex);
    }
    // Add the stripe to the buffer
    rb->buffer[rb->write_idx] = *stripe;
    rb->write_idx = (rb->write_idx + 1) % NUM_STRIPES;
    rb->count++;
    //pthread_cond_signal(&rb->cond);
    //pthread_mutex_unlock(&rb->mutex);
    
}

// Get an item from the ring buffer
void get_from_buffer(ring_buffer_t *rb, stripe_t *stripe) {
    //pthread_mutex_lock(&rb->mutex);
    
    if(rb->count)
    {
    // Wait if the buffer is empty
    while (rb->count == 0) {
        //pthread_cond_wait(&rb->cond, &rb->mutex);
    }
    // Get the stripe from the buffer
    *stripe = rb->buffer[rb->read_idx];
    rb->read_idx = (rb->read_idx + 1) % NUM_STRIPES;
    rb->count--;
    //pthread_cond_signal(&rb->cond);
    //pthread_mutex_unlock(&rb->mutex);
    //printf("g_unlocked mutex\n");
    }
}

int byteCnt=0;

// Reader thread function
void *reader_thread(void *arg) {
    threadArgsType *threadarg = (threadArgsType *)arg;
    ring_buffer_t *rb = threadarg->r_buffer;

    stripe_t stripe;
    int done = 0;

    int fd[5], idx;
    FILE *fdin;
    unsigned char stripe_buf[5*512];
    int offset=0, bread=0, btoread=(4*512), bwritten=0, btowrite=(512), sectorCnt=0;
    int thread_ind;

    fdin = fopen(threadarg->inputFileName, "r");
    fd[0] = open("StripeChunk1.bin", O_RDWR | O_CREAT, 00644);
    fd[1] = open("StripeChunk2.bin", O_RDWR | O_CREAT, 00644);
    fd[2] = open("StripeChunk3.bin", O_RDWR | O_CREAT, 00644);
    fd[3] = open("StripeChunk4.bin", O_RDWR | O_CREAT, 00644);
    fd[4] = open("StripeChunkXOR.bin", O_RDWR | O_CREAT, 00644);

    while(!abortS1)
    {
        sem_wait(&semS1);
        if(abortS1) break;

        if( threadarg->r_buffer->count == NUM_STRIPES)
        continue;

        // read a stripe or to end of file
        offset=0, bread=0, btoread=(4*512);
        do
        {
            bread=fread(&stripe_buf[offset], 1, btoread, fdin); 
            offset+=bread;
            btoread=(4*512)-bread;
        }
        while (!(feof(fdin)) && (btoread > 0));

        //Now split the buffer into the 4 parts of the stripe
        memcpy(stripe.LBA1, stripe_buf, SECTOR_SIZE);
        memcpy(stripe.LBA2, stripe_buf + SECTOR_SIZE, SECTOR_SIZE);
        memcpy(stripe.LBA3, stripe_buf + 2 * SECTOR_SIZE, SECTOR_SIZE);
        memcpy(stripe.LBA4, stripe_buf + 3 * SECTOR_SIZE, SECTOR_SIZE);


        if((offset < (4*512)) && (feof(fdin)))
        {
            write_trace("hit end of file\n");
            bzero(&stripe_buf[offset], btoread);
            byteCnt+=offset;
        }
        else
        {
            write_trace("read full stripe\n");
            assert(offset == (4*512));
            byteCnt+=(4*512);
        };

        // Add the read stripe to the ring buffer
        add_to_buffer(rb, &stripe);

        sectorCnt+=4;
        done = feof(fdin);
        if(done)
            break;
    }

    fclose(fdin);

    readDone = 1;

    return byteCnt;
}

// XOR thread function
void *xor_thread(void *arg) {
    threadArgsType *threadarg = (threadArgsType *)arg;

    while(!abortS2)
    {
        sem_wait(&semS2);
        if(abortS2) break;
        stripe_t stripe;
        // Get a stripe from the buffer
        if(threadarg->r_buffer->count == 0)
            continue;
        get_from_buffer(threadarg->r_buffer, &stripe);

        // Perform XOR on the stripe
        xorLBA(stripe.LBA1, stripe.LBA2, stripe.LBA3, stripe.LBA4, stripe.PLBA);
        //printf("did xor\n");
        // Add the processed stripe back to the buffer (ready for writing)
        add_to_buffer(threadarg->w_buffer, &stripe);
        if(readDone)
            if(threadarg->r_buffer->count == 0)
                break;
    } 
    xorDone = 1;
    return NULL;
}

// Writer thread function
void *writer_thread(void *arg) {
    threadArgsType *threadarg = (threadArgsType *)arg;
    ring_buffer_t *rb = threadarg->w_buffer;
    int fd[5];
    int offset=0, bwritten=0, btowrite=(512);

    fd[0] = open("StripeChunk1.bin", O_RDWR | O_CREAT, 00644);
    fd[1] = open("StripeChunk2.bin", O_RDWR | O_CREAT, 00644);
    fd[2] = open("StripeChunk3.bin", O_RDWR | O_CREAT, 00644);
    fd[3] = open("StripeChunk4.bin", O_RDWR | O_CREAT, 00644);
    fd[4] = open("StripeChunkXOR.bin", O_RDWR | O_CREAT, 00644);

    while(!abortS3)
    {
        sem_wait(&semS3);
        if(abortS3) break;

        stripe_t stripe;

        if(threadarg->w_buffer->count == 0)
            continue;

        // Get a stripe from the buffer
        get_from_buffer(threadarg->w_buffer, &stripe);
        // write out the stripe + xor code
        //
        offset=0, bwritten=0, btowrite=(512);
        do
        {
            bwritten=write(fd[0], &stripe.LBA1[0], 512); 
            offset+=bwritten;
            btowrite=(512)-bwritten;
        }
        while (btowrite > 0);

        offset=512, bwritten=0, btowrite=(512);
        do
        {
            bwritten=write(fd[1], &stripe.LBA2[0], 512); 
            offset+=bwritten;
            btowrite=(512)-bwritten;
        }
        while (btowrite > 0);

        offset=1024, bwritten=0, btowrite=(512);
        do
        {
            bwritten=write(fd[2], &stripe.LBA3[0], 512); 
            offset+=bwritten;
            btowrite=(512)-bwritten;
        }
        while (btowrite > 0);

        offset=1536, bwritten=0, btowrite=(512);
        do
        {
            bwritten=write(fd[3], &stripe.LBA4[0], 512); 
            offset+=bwritten;
            btowrite=(512)-bwritten;
        }
        while (btowrite > 0);

        offset=2048, bwritten=0, btowrite=(512);
        do
        {
            bwritten=write(fd[4], &stripe.PLBA[0], 512); 
            offset+=bwritten;
            btowrite=(512)-bwritten;
        }
        while (btowrite > 0);
        //printf("did write\n");
        if( xorDone)
            if(threadarg->w_buffer->count == 0)
                break;

    } 

    for(int idx=0; idx < 5; idx++) close(fd[idx]);

    return NULL;
}


// returns bytes written or ERROR code
// 
int stripeFileThread(char *inputFileName, int offsetSectors)
{
    void *threadReturn;
    int * retval;
    pthread_t reader, xor, writer;
    threadArgsType readerParam, xorParam, writerParam;

    //cpu params
    cpu_set_t threadcpu;
    cpu_set_t allcpuset;

    pthread_attr_t rt_sched_attr[NUM_THREADS];
    int rt_max_prio, rt_min_prio, cpuidx;

    struct sched_param rt_param[NUM_THREADS];
    struct sched_param main_param;
    int scope, rc, flags=0;

    pthread_attr_t main_attr;
    pid_t mainpid;

    CPU_ZERO(&allcpuset);
    //get cpu avail
    for(int i=0; i < NUM_CPU_CORES; i++)
       CPU_SET(i, &allcpuset);

    // initialize the sequencer semaphores
    //
    if (sem_init (&semS1, 0, 0)) { printf( "Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf( "Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { printf( "Failed to initialize S3 semaphore\n"); exit (-1); }

    mainpid=getpid();
    //set priorities
    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);
    //set up sched fifp
    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");

    //get pthread scope
    pthread_attr_getscope(&main_attr, &scope);

    

    //create threads' attr: fifo, explicit sched, cpu 3
    for(int i=0; i < NUM_THREADS; i++)
    {
        CPU_ZERO(&threadcpu);
        cpuidx=(3);
        CPU_SET(cpuidx, &threadcpu);

      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

      rt_param[i].sched_priority=rt_max_prio-i;
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);
    }

    // Initialize ring buffer
    ring_buffer_t r_rb, w_rb;
    init_ring_buffer(&r_rb);
    init_ring_buffer(&w_rb);
    readDone = 0;
    xorDone = 0;

    //create params
    readerParam.thread_idx = 0;
    readerParam.inputFileName = inputFileName;
    readerParam.r_buffer = &r_rb;
    readerParam.w_buffer = &w_rb;
    xorParam.thread_idx = 0;
    xorParam.inputFileName = inputFileName;
    xorParam.r_buffer = &r_rb;
    xorParam.w_buffer = &w_rb;
    writerParam.thread_idx = 0;
    writerParam.inputFileName = inputFileName;
    writerParam.r_buffer = &r_rb;
    writerParam.w_buffer = &w_rb;

    // Create threads
    rt_param[0].sched_priority=rt_max_prio-1;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
    pthread_create(&reader, &rt_sched_attr[0], reader_thread, &readerParam);
    //sleep(1);//let reader get going
    rt_param[1].sched_priority=rt_max_prio-3;
    pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
    pthread_create(&xor, &rt_sched_attr[1], xor_thread, &xorParam);

    rt_param[2].sched_priority=rt_max_prio-2;
    pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
    pthread_create(&writer, &rt_sched_attr[2], writer_thread, &writerParam);

    timer_create(CLOCK_REALTIME, NULL, &timer_1);
    signal(SIGALRM, (void(*)()) Sequencer);
    /* arm the interval timer */
    itime.it_interval.tv_sec = 0;
    itime.it_interval.tv_nsec = 25000;
    itime.it_value.tv_sec = 0;
    itime.it_value.tv_nsec = 25000;
    timer_settime(timer_1, flags, &itime, &last_itime);
    //set start time


    // Wait for all threads to finish (this example runs indefinitely)
    pthread_join(reader, NULL);
    pthread_join(xor, NULL);
    pthread_join(writer, NULL);

    retval = (int*)threadReturn;

    // Clean up
    pthread_mutex_destroy(&r_rb.mutex);
    pthread_cond_destroy(&r_rb.cond);
    pthread_mutex_destroy(&w_rb.mutex);
    pthread_cond_destroy(&w_rb.cond);       
    

    return byteCnt;
}


void Sequencer(int id)
{
    struct timespec current_time_val;
    double current_realtime;
    int rc, flags=0;
    

    // received interval timer signal
           
    seqCnt++;

    // clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    // printf("Sequencer on core %d for cycle %llu @ sec=%6.9lf\n", sched_getcpu(), seqCnt, current_realtime-start_realtime);
    // syslog(LOG_CRIT, "Sequencer on core %d for cycle %llu @ sec=%6.9lf\n", sched_getcpu(), seqCnt, current_realtime-start_realtime);


    // Release each service at a sub-rate of the generic sequencer rate

    // Servcie_1 = RT_MAX-1	@ 50 Hz
    if((seqCnt % 2) == 1) sem_post(&semS1);

    // Servcie_2 = RT_MAX-2	@ 20 Hz
    if((seqCnt % 5) == 1) sem_post(&semS2);

    // Servcie_3 = RT_MAX-3	@ 10 Hz
    if((seqCnt % 3) == 1 ) sem_post(&semS3);




    
    if(abortTest)
    {
        // disable interval timer
        itime.it_interval.tv_sec = 0;
        itime.it_interval.tv_nsec = 0;
        itime.it_value.tv_sec = 0;
        itime.it_value.tv_nsec = 0;
        timer_settime(timer_1, flags, &itime, &last_itime);
	//syslog(LOG_CRIT, "Disabling sequencer interval timer with abort=%d and %llu of %lld\n", abortTest, seqCnt, sequencePeriods);

	// shutdown all services
        sem_post(&semS1); sem_post(&semS2); sem_post(&semS3);

        abortS1=TRUE; abortS2=TRUE; abortS3=TRUE;
    }

}



void traceOn(void)
{
    printTrace=1;
    tracefile = fopen("tracefile.txt", "w");
}

void traceOff(void)
{
    printTrace=0;
}

// write a trace to the file
void write_trace(const char *format, ...) {
    va_list args;
    va_start(args, format);  // Initialize the va_list with the last known fixed argument
        
    if (tracefile != NULL) {
        vfprintf(tracefile, format, args);  // Write the formatted string to the file

        va_end(args);  // Clean up the va_list
    } else {
        vprintf(format, args);  // Write the formatted string to the console
    }
}

// close the file when done
void close_file() {
    if (tracefile != NULL) {
        fclose(tracefile);  // Close the file
    }
}