#ifndef CAPTURELIB
#define CAPTURELIB


/// @brief desired speed in hz
extern int speed_hz;

/// @brief if not 0, laplace enabled, else rgb image only
extern int en_laplace;

/// @brief if not 0, save diff img, overridden by en_laplace
extern int en_diff_img;

/// @brief initialized camera, creates buffers for reads
void init();

/// @brief uninitialize camera
void uninit();

/// @brief gets frame from camera, runs grayscale and puts new frame in new frame queue
void capture();

/// @brief checks for frame in new frame queue. If available, performs diff then puts new frame into post processing queue
void performDiff();

/// @brief checks for frame in post processing queue. If available, performs post process then puts new frame into save queue
void postProcess();


/// @brief saves available image in save queue
/// @return number of frames saved
int  saveImg();

#endif