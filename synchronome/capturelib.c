/*
 *
 *  Adapted by Sam Siewert for use with UVC web cameras and Bt878 frame
 *  grabber NTSC cameras to acquire digital video from a source,
 *  time-stamp each frame acquired, save to a PGM or PPM file.
 *
 *  The original code adapted was open source from V4L2 API and had the
 *  following use and incorporation policy:
 * 
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 *
 * sudo apt-get install sysstat
 *
 */
//ignore: ‘strncat’ specified bound 5 equals source length [-Wstringop-overflow=] warnings
//used to format headers only
#pragma GCC diagnostic ignored "-Wstringop-overflow"

#include "capturelib.h"

#include <syslog.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include <time.h>

#include <pthread.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"
#define GRAY_SIZE (640*480) //only contains light intensity component - 1 byte per pixel
#define RGB_SIZE (640*480*3)//rgb has 3 bytes per pixel

#define START_UP_FRAMES (25) //found by test
#define LAST_FRAMES (1)
//#define CAPTURE_FRAMES (1800+LAST_FRAMES)
#define CAPTURE_FRAMES (15+LAST_FRAMES)
#define FRAMES_TO_ACQUIRE (CAPTURE_FRAMES + START_UP_FRAMES + LAST_FRAMES)

static int frames_per_sec = 2;

#define COLOR_CONVERT_RGB //color convert image
#define DUMP_FRAMES //save frames
#define DUMP_PPM //save with color pixels
//#define TRANSFORM_IMG //apply laplace
#define DIFF //apply diff

#define BUFFER_SIZE 10  // circular buffer max size

// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;

//io read methods
enum io_method 
{
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
};

//new camera frame ptr object
//used to get frame from camera
struct buffer 
{
    void   *start;
    size_t  length;
};


//frame object
struct frame
{
    struct v4l2_buffer *buf;                //original camera frame
    unsigned char gray_frame[GRAY_SIZE];    //gray scaled image frame
    unsigned char rgb_frame[RGB_SIZE];   //rgb applied frame
    unsigned char laplace_frame[GRAY_SIZE]; //laplace applied frame
};

// Circular buffer structure
struct circular_buffer {
    struct frame *buffer[BUFFER_SIZE]; // Array of frame pointers
    int head;
    int tail;
    int count;
    pthread_mutex_t mutex; // Mutex for thread safety
};

static struct circular_buffer new_frame_cb;
static struct circular_buffer post_proc_cb;
static struct circular_buffer save_frame_cb;

static char            *dev_name;               //device name
static enum io_method   io = IO_METHOD_MMAP;    //memory mapped reads
static int              fd = -1;                //file descripter
struct buffer          *buffers;                //memory mapped buffers for new frames from camera
static unsigned int     n_buffers;              //number of buffers allocated
static int              force_format=1;         //force format of images from camera

static int              frame_count = (FRAMES_TO_ACQUIRE);  //frame count to acquire

//time keeping variables for logging
static double fnow=0.0, fstart=0.0, fstop=0.0;
static struct timespec time_now, time_start, time_stop;

static double fcur=0.0, t1=0.0, t2=0.0;
static struct timespec time_cur, time1, time2;

/////////////////////////// circular buffer ///////////////////////////

//initialize circular buffer
static void init_circular_buffer(struct circular_buffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
    pthread_mutex_init(&cb->mutex, NULL);
}

// create and initialize a frame
static struct frame *create_frame() {
    struct frame *new_frame = malloc(sizeof(struct frame));
    if (!new_frame) {
        perror("Failed to allocate frame");
        exit(EXIT_FAILURE);
    }

    // Allocate memory for image frames
    new_frame->buf = malloc(sizeof(struct v4l2_buffer));
    if (!new_frame->buf) {
        perror("Failed to allocate memory for v4l2_buffer");
        free(new_frame);  // free the frame memory before exiting
        exit(EXIT_FAILURE);
    }

    return new_frame;
}


/// @brief adds f to to cb
/// @param cb circular buffer
/// @param f frame to add
/// @return 0 if successful
static int add_to_buffer(struct circular_buffer *cb, struct frame *f) {
    pthread_mutex_lock(&cb->mutex);

    if (cb->count == BUFFER_SIZE) {
        printf("Buffer full, cannot add frame.\n");
        pthread_mutex_unlock(&cb->mutex);
        return -1;  // Buffer full
    }

    cb->buffer[cb->head] = f;
    cb->head = (cb->head + 1) % BUFFER_SIZE;
    cb->count++;

    pthread_mutex_unlock(&cb->mutex);
    return 0;
}

/// @brief gets frame from circular buffer
/// @param cb circular buffer
/// @return ptr to frame removed
struct frame *remove_from_buffer(struct circular_buffer *cb) {
    pthread_mutex_lock(&cb->mutex);

    if (cb->count == 0) {
        //printf("Buffer empty, cannot remove frame.\n");
        pthread_mutex_unlock(&cb->mutex);
        return NULL; // Buffer empty
    }

    struct frame *f = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % BUFFER_SIZE;
    cb->count--;

    pthread_mutex_unlock(&cb->mutex);
    return f;
}


/// @brief cleanup frame
/// @param f 
static void free_frame(struct frame *f) {
    if (!f) return;
    free(f->buf);  // free the allocated buffer
    free(f);       // free the frame object
}

/// @brief cleanup buffer
/// @param cb 
static void free_circular_buffer(struct circular_buffer *cb) {
    struct frame *f;
    while( ( f = remove_from_buffer(cb) ) != NULL )
        free_frame(f); // free each frame removed from the buffer

    pthread_mutex_destroy(&cb->mutex); // clean up the mutex
}

/////////////////////////// end circular buffer ///////////////////////////

// 3x3 Laplacian kernel (4-connectivity) for difference detection
static int laplacian_kernel[3][3] = {
    { 0,  1,  0},
    { 1, -4,  1},
    { 0,  1,  0}
};

//exit with error code
static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}
//buffer control to camera
static int xioctl(int fh, int request, void *arg)
{
        int r;

        do 
        {
            r = ioctl(fh, request, arg);

        } while (-1 == r && EINTR == errno);

        return r;
}

//file name headers
static char ppm_header[]="P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
static char ppm_dumpname[]="frames/test0000.ppm";
//save image as ppm (with rgb color)
static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&ppm_dumpname[11], 9, "%04d", tag);
    strncat(&ppm_dumpname[15], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&ppm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    // subtract 1 from sizeof header because it includes the null terminator for the string
    written=write(dumpfd, ppm_header, sizeof(ppm_header)-1);

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    //printf("Frame written to flash at %lf, %d, bytes\n", (fnow-fstart), total);

    close(dumpfd);
    
}

//file name headers
static char pgm_header[]="P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
static char pgm_dumpname[]="frames/test0000.pgm";
//save image as pgm (grey scale uvuv)
static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&pgm_dumpname[11], 9, "%04d", tag);
    strncat(&pgm_dumpname[15], ".pgm", 5);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&pgm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    // subtract 1 from sizeof header because it includes the null terminator for the string
    written=write(dumpfd, pgm_header, sizeof(pgm_header)-1);

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    clock_gettime(CLOCK_MONOTONIC, &time_now);
    fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
    //printf("Frame written to flash at %lf, %d, bytes\n", (fnow-fstart), total);

    close(dumpfd);
    
}

//color convert image to get rgb component
static void yuv2rgb_float(float y, float u, float v, 
                   unsigned char *r, unsigned char *g, unsigned char *b)
{
    float r_temp, g_temp, b_temp;

    // R = 1.164(Y-16) + 1.1596(V-128)
    r_temp = 1.164*(y-16.0) + 1.1596*(v-128.0);  
    *r = r_temp > 255.0 ? 255 : (r_temp < 0.0 ? 0 : (unsigned char)r_temp);

    // G = 1.164(Y-16) - 0.813*(V-128) - 0.391*(U-128)
    g_temp = 1.164*(y-16.0) - 0.813*(v-128.0) - 0.391*(u-128.0);
    *g = g_temp > 255.0 ? 255 : (g_temp < 0.0 ? 0 : (unsigned char)g_temp);

    // B = 1.164*(Y-16) + 2.018*(U-128)
    b_temp = 1.164*(y-16.0) + 2.018*(u-128.0);
    *b = b_temp > 255.0 ? 255 : (b_temp < 0.0 ? 0 : (unsigned char)b_temp);
}


// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

static void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}

// Apply the Laplacian filter
static void apply_laplacian(const unsigned char *input, unsigned char *output, int width, int height)
{
    for (int y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            int sum = 0;

            // Apply the 3x3 Laplacian kernel
            for (int ky = -1; ky <= 1; ky++) {
                for (int kx = -1; kx <= 1; kx++) {
                    int pixel = input[(y + ky) * width + (x + kx)];
                    sum += pixel * laplacian_kernel[ky + 1][kx + 1];
                }
            }

            // Normalize to [0, 255] range
            if (sum < 0) sum = 0;
            if (sum > 255) sum = 255;

            output[y * width + x] = (unsigned char)sum;
        }
    }
}

//detect motion (difference) between 2 images: prev_img and curr_img. save result in diffed_img
static int detect_motion(unsigned char *prev_img, unsigned char *curr_img, int size, unsigned char *diffed_img) {
    int threshold = 10;  // Example threshold to ignore small differences
    int non_zero_count = 0;
    
    // Subtract images
    for (int i = 0; i < size; ++i) {
        int diff = abs(curr_img[i] - prev_img[i]);
        diffed_img[i] = (diff > threshold) ? 255 : 0;  // Apply threshold
    }

    // Count non-zero pixels (motion detected areas)
    for (int i = 0; i < size; ++i) {
        if (diffed_img[i] != 0) {
            non_zero_count++;
        }
    }
    printf("different pixes: %d\n", non_zero_count);
    // If there are significant differences, consider motion detected
    if (non_zero_count > (2000)) {  // more than 2000 of pixels differ (found by test)
        //printf("Motion detected\n");
        return 1;
    } else {
       // printf("No motion detected\n");
        return 0;
    }
}

// always ignore first framecnt frames
int framecnt=-START_UP_FRAMES;
//scratchpad buffer for conversion
unsigned char bigbuffer[(1280*960)];

unsigned char last_image[(1280*960)]; //buffer for previous image
unsigned char diff_image[(1280*960)]; //buffer for diff'd image

//process the image
static void process_image(const void *p, int size)
{
    int i, newi, newsize=0;
    struct timespec frame_time;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    //printf("frame %d: ", framecnt);
    
    if(framecnt == 0) 
    {
        clock_gettime(CLOCK_MONOTONIC, &time_start);
        fstart = (double)time_start.tv_sec + (double)time_start.tv_nsec / 1000000000.0;
    }

#ifdef DUMP_FRAMES	

    // This just dumps the frame to a file now, but you could replace with whatever image
    // processing you wish.
    //

    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) //we are using this one
    {

#if defined(COLOR_CONVERT_RGB)
       
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want RGB, so RGBRGB which is 6 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+6)
        {
            y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
            yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[newi], &bigbuffer[newi+1], &bigbuffer[newi+2]);
            yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[newi+3], &bigbuffer[newi+4], &bigbuffer[newi+5]);
        }

#endif


#if defined(DUMP_PPM)
        if(framecnt > -1) 
        {
            dump_ppm(bigbuffer, ((size*6)/4), framecnt, &frame_time);
            //printf("Dump YUYV converted to RGB size %d\n", size);
        }
#else
      
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want Y, so YY which is 2 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+2)
        {
            // Y1=first byte and Y2=third byte
            bigbuffer[newi]=pptr[i];
            bigbuffer[newi+1]=pptr[i+2];
        }

        if(framecnt > -1)
        {
            dump_pgm(bigbuffer, (size/2), framecnt, &frame_time);
            //printf("Dump YUYV converted to YY size %d\n", size);
        }
#endif

#if defined(TRANSFORM_IMG)
    if(framecnt > -1) 
    {
        int width = 1280; //buffer size allocation
        int height = 960;

        // Create buffer for grayscale (Y channel)
        unsigned char *y_plane = malloc(width * height);
        if (!y_plane) {
            perror("Failed to allocate memory for Y plane");
            exit(EXIT_FAILURE);
        }

        // Extract Y component (grayscale image)
        for (i = 0, newi = 0; i < size; i += 4, newi += 2) {
            // Y1 = first byte, Y2 = third byte
            y_plane[newi] = pptr[i];     // First Y
            y_plane[newi + 1] = pptr[i + 2]; // Second Y
        }

        // Allocate buffer for the Laplacian output
        unsigned char *laplacian_output = malloc(width * height);
        if (!laplacian_output) {
            perror("Failed to allocate memory for Laplacian output");
            free(y_plane);
            exit(EXIT_FAILURE);
        }

        // Apply Laplacian edge detection
        apply_laplacian(y_plane, laplacian_output, width, height);

        // for (int i = 0; i < width * height; i++) {
        //     laplacian_output[i] = 255 - laplacian_output[i];  // Invert pixel intensity
        // }

        // Save as a PGM image (for visualization/debugging)
        dump_pgm(laplacian_output, width * height, framecnt, &frame_time);

        free(y_plane);
        free(laplacian_output);
    }
#endif

#if defined(DIFF)
    //test time: start
    clock_gettime(CLOCK_MONOTONIC, &time1);
    t1 = (double)time1.tv_sec + (double)time1.tv_nsec / 1000000000.0;

    //get grey scale image
    for(i=0, newi=0; i<size; i=i+4, newi=newi+2)
    {
        // Y1=first byte and Y2=third byte
        bigbuffer[newi]=pptr[i];
        bigbuffer[newi+1]=pptr[i+2];
    }

    //test time: end
    clock_gettime(CLOCK_MONOTONIC, &time2);
    t2 = (double)time2.tv_sec + (double)time2.tv_nsec / 1000000000.0;
    syslog(LOG_CRIT, "TIMETRACE: read time %lf\n", (t2-t1));

    if(framecnt > -1)
    {
        //diff images
        detect_motion(last_image, bigbuffer, size, diff_image);

        //save diffed image as greyscale
        dump_pgm(diff_image, (size/2), framecnt, &frame_time);
    }

    //copy new image
    memcpy(last_image, bigbuffer, size);
#endif

    }

    else
    {
        printf("ERROR - unknown dump format\n");
    }

#endif

}



/// @brief applies rgb to image at *p and saves to *rgb_img with size 640x480x3 max
/// @param p orignal frame from camera location
/// @param size size of original frame
/// @param rgb_img new rgb image - assumes already allocated memory!
static void apply_rgb(const void *p, int size, unsigned char *rgb_img)
{
    int i, newi=0;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    if( size > (640*480*2) )//max size of original image
    {
        perror("Attempt to rgb too large image");
        exit(EXIT_FAILURE);
    }

    if( rgb_img == NULL )
    {
        perror("unallocated rgb_img");
        exit(EXIT_FAILURE);
    }

    // Pixels are YU and YV alternating, so YUYV which is 4 bytes
    // We want RGB, so RGBRGB which is 6 bytes
    //
    for(i=0, newi=0; i<size; i=i+4, newi=newi+6)
    {
        y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
        yuv2rgb(y_temp, u_temp, v_temp, &rgb_img[newi], &rgb_img[newi+1], &rgb_img[newi+2]);
        yuv2rgb(y2_temp, u_temp, v_temp, &rgb_img[newi+3], &rgb_img[newi+4], &rgb_img[newi+5]);
    }
}

/// @brief applies grayscaling to image at *p and saves to *gray_img with size 640x480 max
/// @param p orignal frame from camera location
/// @param size size of original frame
/// @param gray_img new grayscale image - assumes already allocated memory!
static void apply_grayscale(const void *p, int size, unsigned char *gray_img)
{
    unsigned char *pptr = (unsigned char *)p;

    if( size > (640*480*2) )//max size of original image
    {
        perror("Attempt to gray scale too large image");
        exit(EXIT_FAILURE);
    }

    if( gray_img == NULL )
    {
        perror("unallocated gray_img");
        exit(EXIT_FAILURE);
    }

    // Extract Y component (grayscale image)
    for (int i = 0, newi = 0; i < size; i += 4, newi += 2) {
        // Y1 = first byte, Y2 = third byte
        gray_img[newi] = pptr[i];     // First Y
        gray_img[newi + 1] = pptr[i + 2]; // Second Y
    }
}

//get frame from camera buffers
static int read_frame(void)
{
    struct v4l2_buffer buf; //working image buffer
    unsigned int i;
    struct frame *f1;//frame object
    
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) //get image (dequeue buffer) from camera, save ptr to buf
    {
        switch (errno) //error handle
        {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                    non-fatal errors too.
                    */
                return 0;

            default:
                printf("mmap failure\n");
                errno_exit("VIDIOC_DQBUF");
        }
    }
    //valid buffer?
    assert(buf.index < n_buffers);

    //create buffer obj
    f1 = create_frame();
    //copy camera frame to new frame object
    memcpy(f1->buf, &buf, sizeof(struct v4l2_buffer));
    //apply grayscale
    apply_grayscale(buffers[f1->buf->index].start, f1->buf->bytesused, f1->gray_frame);
    //add new frame to new frame buffer
    add_to_buffer(&new_frame_cb, f1);

    // //process the image
    // process_image(buffers[buf.index].start, buf.bytesused);
    // //release (queue buffer) image buffer back to camera for further use
    // if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
    //         errno_exit("VIDIOC_QBUF");

    return 1;
}


static void mainloop(void)
{
    unsigned int count;
    struct timespec read_delay;
    struct timespec time_error;

    
#if (FRAMES_PER_SEC  == 1)
    printf("Running at 1 frame/sec\n");
    read_delay.tv_sec=1;
    read_delay.tv_nsec=0;
#elif (FRAMES_PER_SEC == 2)
    printf("Running at 2 frames/sec\n");
    read_delay.tv_sec=0;
    read_delay.tv_nsec=500000000;
#elif (FRAMES_PER_SEC == 10)
    printf("Running at 10 frames/sec\n");
    read_delay.tv_sec=0;
    read_delay.tv_nsec=100000000;
#elif (FRAMES_PER_SEC == 20)
    printf("Running at 20 frames/sec\n");
    read_delay.tv_sec=0;
    read_delay.tv_nsec=49625000;
#elif (FRAMES_PER_SEC == 25)
    printf("Running at 25 frames/sec\n");
    read_delay.tv_sec=0;
    read_delay.tv_nsec=39625000;
#elif (FRAMES_PER_SEC == 30)
    printf("Running at 30 frames/sec\n");
    read_delay.tv_sec=0;
    read_delay.tv_nsec=33333333;
#else
    printf("Running at 1 frame/sec\n");
    read_delay.tv_sec=1;
    read_delay.tv_nsec=0;
#endif

    count = frame_count;

    while (count > 0)
    {
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(fd + 1, &fds, NULL, NULL, &tv); //get file descripter
            //error handle
            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }
            //read frame
            if (read_frame())
            {
                if(nanosleep(&read_delay, &time_error) != 0) //wait read period
                    perror("nanosleep");
                else
                {
                    if(framecnt > 1) //log
                    {	
                        clock_gettime(CLOCK_MONOTONIC, &time_now);
                        fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / 1000000000.0;
                                //printf("REPLACE read at %lf, @ %lf FPS\n", (fnow-fstart), (double)(framecnt+1) / (fnow-fstart));
                                syslog(LOG_CRIT, "SIMPCAP: read at %lf, @ %lf FPS\n", (fnow-fstart), (double)(framecnt+1) / (fnow-fstart));
                    }
                }

                count--;
                break;
            }

            /* EAGAIN - continue select loop unless count done. */
            if(count <= 0) break;
        }

        if(count <= 0) break;
    }

    clock_gettime(CLOCK_MONOTONIC, &time_stop);
    fstop = (double)time_stop.tv_sec + (double)time_stop.tv_nsec / 1000000000.0;

}
//stop capturing
static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");
}
//start capturing
static void start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < n_buffers; ++i) 
    {
        printf("allocated buffer %d\n", i);
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) //allocate mem map buffer
            errno_exit("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)) //set buffer for video stream
        errno_exit("VIDIOC_STREAMON");
}

//init camera
static void uninit_device(void)
{
    unsigned int i;

    for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap(buffers[i].start, buffers[i].length)) //unmap the buffers
            errno_exit("munmap");

    free(buffers);
}

//init mem mapping
static void init_mmap(void)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 6; //allocate 6 buffers for reading
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; //of type video capture
    req.memory = V4L2_MEMORY_MMAP; //memory map type memory

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) //request the buffers from camera
    {
            if (EINVAL == errno) 
            {
                fprintf(stderr, "%s does not support "
                            "memory mapping\n", dev_name);
                exit(EXIT_FAILURE);
            } else 
            {
                errno_exit("VIDIOC_REQBUFS");
            }
    }

    if (req.count < 2) //not enough buffers
    {
        fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
        exit(EXIT_FAILURE);
    }

    buffers = calloc(req.count, sizeof(*buffers)); //allocate buffer size

    if (!buffers) 
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    //initialize each buffer
    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                errno_exit("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start =
                mmap(NULL /* start anywhere */,
                        buf.length,
                        PROT_READ | PROT_WRITE /* required */,
                        MAP_SHARED /* recommended */,
                        fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start)
                errno_exit("mmap");
    }
}

//init camera
static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) //get capability
    {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                     dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
                errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) //get device
    {
        fprintf(stderr, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n",
                    dev_name);
        exit(EXIT_FAILURE);
    }


    /* Select video input, video standard and tune here. */
    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                        break;
            }
        }

    }
    else
    {
        /* Errors ignored. */
    }

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    //setup format of images
    if (force_format)
    {
        //printf("FORCING FORMAT\n");
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here
        // This one works for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                    errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    init_mmap();
}

//close device
static void close_device(void)
{
    if (-1 == close(fd))
            errno_exit("close");

    fd = -1;
}

//open device
static void open_device(void)
{
    struct stat st;

    if (-1 == stat(dev_name, &st)) {
            fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                        dev_name, errno, strerror(errno));
            exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode)) {
            fprintf(stderr, "%s is no device\n", dev_name);
            exit(EXIT_FAILURE);
    }

    fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd) {
            fprintf(stderr, "Cannot open '%s': %d, %s\n",
                        dev_name, errno, strerror(errno));
            exit(EXIT_FAILURE);
    }
}


void init()
{
    dev_name = "/dev/video0";

    init_circular_buffer( &new_frame_cb );
    init_circular_buffer( &post_proc_cb );
    init_circular_buffer( &save_frame_cb );

    // initialization of V4L2
    open_device();
    init_device();
    start_capturing();
}

void uninit()
{
    uninit_device();
    close_device();
    free_circular_buffer( &new_frame_cb );
    free_circular_buffer( &post_proc_cb );
    free_circular_buffer( &save_frame_cb );
}

void capture()
{
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    /* Timeout. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(fd + 1, &fds, NULL, NULL, &tv); //get file descripter
    //error handle
    if (-1 == r)
    {
        if (EINTR == errno)
            return;
        errno_exit("select");
    }

    if (0 == r)
    {
        fprintf(stderr, "select timeout\n");
        exit(EXIT_FAILURE);
    }
    //read frame
    read_frame();
}

void performDiff()
{

}

int framecount = 0;
void saveImg()
{
    struct frame *f1;//frame object
    struct timespec frame_time;
    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);    
    syslog(LOG_CRIT, "save thread start\n");

    f1 = remove_from_buffer(&new_frame_cb);
    syslog(LOG_CRIT, "save thread got frame\n");
    if(f1!=NULL)
    {
        syslog(LOG_CRIT, "save thread actually got frame\n");
        dump_pgm(bigbuffer, GRAY_SIZE, framecount++, &frame_time);
syslog(LOG_CRIT, "save thread save frame\n");
        //release (queue buffer) image buffer back to camera for further use
        if (-1 == xioctl(fd, VIDIOC_QBUF, f1->buf))
            errno_exit("VIDIOC_QBUF");
syslog(LOG_CRIT, "save thread release frame\n");
        free_frame(f1);
        syslog(LOG_CRIT, "save thread free frame\n");
    }


}
