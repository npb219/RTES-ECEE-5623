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

#define START_UP_FRAMES (26) //found by test
#define LAST_FRAMES (1)
//#define CAPTURE_FRAMES (1800+LAST_FRAMES)
#define CAPTURE_FRAMES (15+LAST_FRAMES)
#define FRAMES_TO_ACQUIRE (CAPTURE_FRAMES + START_UP_FRAMES + LAST_FRAMES)

/// @brief desired speed in hz
int speed_hz = 1;

/// @brief if not 0, laplace enabled, else rgb image only
int en_laplace = 0;

/// @brief if not 0, save diff img, overridden by en_laplace
int en_diff_img = 0;


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
    unsigned char rgb_frame[RGB_SIZE];      //rgb applied frame
    unsigned char diff_frame[GRAY_SIZE];    //gray scaled diffed image frame
    unsigned char laplace_frame[GRAY_SIZE]; //laplace applied frame
    double image_capture_start_time;        //capture time
    int in_buffer;
};

// Circular buffer structure
struct circular_buffer {
    struct frame *buffer[BUFFER_SIZE]; // Array of frame pointers
    int head;
    int tail;
    int count;
    pthread_mutex_t mutex; // Mutex for thread safety
};

static struct circular_buffer new_frame_cb; //new captured frame circ buf
static struct circular_buffer post_proc_cb; //unique (diffed) image to be post processed circ buf
static struct circular_buffer save_frame_cb; //post processed image - to be saved circ buf

static char            *dev_name;               //device name
static enum io_method   io = IO_METHOD_MMAP;    //memory mapped reads
static int              fd = -1;                //file descripter
struct buffer          *buffers;                //memory mapped buffers for new frames from camera
static unsigned int     n_buffers;              //number of buffers allocated
static int              force_format=1;         //force format of images from camera

static int              framecount = 0;                         //current frame count acquired and saved
static int              captured_frames = -START_UP_FRAMES;     //current frames captured
static int              diff_sync = 0;                          //counter to sync diff function

static unsigned char prev_frame[GRAY_SIZE];     //previous image - used by service 2 only - grayscaled

//time keeping variables for logging
static double fnow=0.0, fstart=0.0, fstop=0.0;
static struct timespec time_now, time_start, time_stop;

static double fcur=0.0, t1=0.0, t2=0.0;
static struct timespec time_cur, time1, time2;

//timer vals
static struct timespec start_time_val;
static double start_realtime;

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


/// @brief cleanup frame.
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

//get real time as double from timespec
static double realtime(struct timespec *tsptr)
{
    return ((double)(tsptr->tv_sec) + (((double)tsptr->tv_nsec)/1000000000.0));
}

// 3x3 Laplacian kernel (4-connectivity) for difference detection
static int laplacian_kernel[3][3] = {
    { 0,  1,  0},
    { 1, -4,  1},
    { 0,  1,  0}
};

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

/// @brief detect motion (difference) between 2 images: prev_img and curr_img. save result in diffed_img
/// @param prev_img ptr to previous image
/// @param curr_img ptr to current image
/// @param size size of image (bytes)
/// @param diffed_img ptr to save diffed image
/// @return 1 if different, 0 otherwise
static int detect_motion(unsigned char *prev_img, unsigned char *curr_img, int size, unsigned char *diffed_img) {
    int threshold = ( speed_hz == 10 ) ? 69 : 34;//10;  // threshold to ignore small differences
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
    if ( non_zero_count > ( ( speed_hz == 10 ) ? 700 : 430 ) ) {  // more than 800 of pixels differ (found by test)
        //printf("Motion detected\n");
        return 1;
    } else {
       // printf("No motion detected\n");
        return 0;
    }
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
    unsigned int i;
    struct frame *f1;//frame object

    struct timespec frame_time;
    double current_realtime;

    //create buffer obj
    f1 = create_frame();
    
    CLEAR(*f1->buf);

    f1->buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    f1->buf->memory = V4L2_MEMORY_MMAP;

    // record when process was called
    clock_gettime( CLOCK_REALTIME, &frame_time );
    current_realtime=realtime( &frame_time );
    //save frame time
    f1->image_capture_start_time = current_realtime-start_realtime;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, f1->buf)) //get image (dequeue buffer) from camera, save ptr to buf
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
    assert(f1->buf->index < n_buffers);

    
    //apply grayscale
    apply_grayscale(buffers[f1->buf->index].start, f1->buf->bytesused, f1->gray_frame);
    //add new frame to new frame buffer
    add_to_buffer(&new_frame_cb, f1);

    return 1;
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

    req.count = 12; //allocate 6 buffers for reading
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

    //set start time
    clock_gettime(CLOCK_REALTIME, &start_time_val);
    start_realtime=realtime(&start_time_val);

    diff_sync = 0;
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
    static int diff_cnt = 0;
    static int same_cnt = 0;
    
    
    struct frame *f1;//frame object

    struct timespec frame_time;
    double current_realtime;
    // record when process was called
    clock_gettime( CLOCK_REALTIME, &frame_time );
    current_realtime=realtime( &frame_time );

    f1 = remove_from_buffer( &new_frame_cb );
    if( f1!=NULL )
    {
        if( ( captured_frames++ > -1 ) && ( diff_sync < (( speed_hz == 10 ) ? 30 : 5) ) && detect_motion( prev_frame, f1->gray_frame, GRAY_SIZE, f1->diff_frame ) )
        {
            diff_sync++;
            //copy current to previous
            memcpy( prev_frame, f1->gray_frame, GRAY_SIZE );
            
            //release (queue buffer) image buffer back to camera for further use
            if (-1 == xioctl(fd, VIDIOC_QBUF, f1->buf))
                errno_exit("VIDIOC_QBUF");
            //release frame - will not use further
            free_frame( f1 );
        }
        //check frame counter for throwaway. doing here so we still load the diff pipeline
        //then detect motion
        else if( ( captured_frames > -1 ) && detect_motion( prev_frame, f1->gray_frame, GRAY_SIZE, f1->diff_frame ) )
        {
            

            // if(!diff_sync)
            // {
            //     //set start time
            //     clock_gettime(CLOCK_REALTIME, &start_time_val);
            //     start_realtime=realtime(&start_time_val);
            //     diff_sync = 1;
            // }
            // f1->image_capture_start_time = current_realtime-start_realtime;

            //syslog( LOG_CRIT, "S2 diff @ sec=%6.9lf\n", current_realtime-start_realtime );
            //copy current to previous
            memcpy( prev_frame, f1->gray_frame, GRAY_SIZE );

            //add to post process buffer
            add_to_buffer( &post_proc_cb, f1 );
        }
        else
        {
            //syslog( LOG_CRIT, "S2 toss @ sec=%6.9lf\n", current_realtime-start_realtime );
            //copy current to previous
            memcpy( prev_frame, f1->gray_frame, GRAY_SIZE );
            
            //release (queue buffer) image buffer back to camera for further use
            if (-1 == xioctl(fd, VIDIOC_QBUF, f1->buf))
                errno_exit("VIDIOC_QBUF");
            //release frame - will not use further
            free_frame( f1 );
        }
    }
}

void postProcess()
{
    struct frame *f1;//frame object

    struct timespec frame_time;
    double current_realtime;
    // record when process was called
    clock_gettime( CLOCK_REALTIME, &frame_time );
    current_realtime=realtime( &frame_time );

    f1 = remove_from_buffer( &post_proc_cb );
    if( f1!=NULL )
    {
        //syslog( LOG_CRIT, "S3 process @ sec=%6.9lf\n", current_realtime-start_realtime );
        //apply rgb to image
        apply_rgb( buffers[f1->buf->index].start, f1->buf->bytesused, f1->rgb_frame );

        //apply laplace if enabled
        if( en_laplace )
            apply_laplacian( f1->gray_frame, f1->laplace_frame, 640, 480 );

        //add to save buffer
        add_to_buffer( &save_frame_cb, f1 );
    }
}

int saveImg()
{
    struct frame *f1;//frame object
    struct timespec frame_time;
    double current_realtime;
    // record when process was called
    clock_gettime( CLOCK_REALTIME, &frame_time );
    current_realtime = realtime( &frame_time );

    f1 = remove_from_buffer( &save_frame_cb );
    if( f1!=NULL )
    {
        //syslog(LOG_CRIT, "S4 save @ sec=%6.9lf\n", current_realtime-start_realtime);
        syslog( LOG_CRIT, "[Frame Count: %d][Image Capture Start Time: %6.9lf]\n", framecount, f1->image_capture_start_time );

        // dump_pgm( f1->diff_frame, GRAY_SIZE, framecount, &frame_time );
        dump_ppm( f1->rgb_frame, RGB_SIZE, framecount, &frame_time );

        //save laplace if enabled
        if( en_laplace )
            dump_pgm( f1->laplace_frame, GRAY_SIZE, framecount, &frame_time );
        else if( en_diff_img ) //or save diff frame
            dump_pgm( f1->diff_frame, GRAY_SIZE, framecount, &frame_time );
        
        //release (queue buffer) image buffer back to camera for further use
        if (-1 == xioctl(fd, VIDIOC_QBUF, f1->buf))
            errno_exit("VIDIOC_QBUF");
        free_frame( f1 );

        framecount++;
    }

    return framecount;
}
