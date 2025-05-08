/******************************************************************************
* CAM_DRIVER.H
* 
* This header file defines the interface for controlling the OLED display
* with camera input using the SSD1351 controller. It provides functions for:
*   - Initializing and cleaning up the OLED display
*   - Converting YUV420 colorspace to RGB for display
*   - Playing back pre-recorded YUV420 video files
*   - Displaying real-time camera feed from a named pipe
*   - Managing display status and playback controls
*
* The driver is designed for a 128x128 pixel OLED display connected via SPI
* and operates at 30 frames per second. It reads camera data in YUV420 format,
* converts it to appropriate display format, and manages timing for smooth
* video playback.
*
* Author: The One Project is Real
* Date: 02/19/2025
*
* License: Version 2.1, February 1999
*
* Copyright (C) 1991, 1999 Free Software Foundation, Inc.
* <https://fsf.org/>
* Everyone is permitted to copy and distribute verbatim copies
* of this license document, but changing it is not allowed.

* [This is the first released version of the Lesser GPL.  It also counts
*  as the successor of the GNU Library Public License, version 2, hence
*  the version number 2.1.]
*
******************************************************************************/
#ifndef CAM_DRIVER_H
#define CAM_DRIVER_H

#include <stdint.h>             // for uint8_t (8-bit unsigned integer)
#include <stddef.h>             // defines size_t

//FPS setting (must match main's FPS)
#define FPS 12

// OLED display dimensions
#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 128
#define PIPE_PATH "/tmp/stream_pipe"

//UBYTE *oled_buffer;

/******************************************************************************
 * Initialize the OLED display.
 * 
 * This function sets up the SPI interface and configures the display
 * for proper operation with the Waveshare 1.5" OLED with SSD1327 controller.
 * 
 * returns 0 on success, -1 on failure
 *******************************************************************************/
 int oled_init(void);

 /******************************************************************************
 * Clean up OLED display resources.
 * 
 * This function should be called before program exit to properly
 * shut down the display and free hardware resources.
 *******************************************************************************/
void oled_cleanup(void);

/******************************************************************************
 * Convert YUV420 color space to RGB. [colorspace conversion]
 * 
 * param y - Y component
 * param u - U component
 * param v - V component
 * param r Pointer to store R component
 * param g Pointer to store G component
 * param b Pointer to store B component
 *******************************************************************************/
void yuv420_to_rgb(uint8_t y, uint8_t u, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);

/******************************************************************************
 * Start video playback on the OLED display.
 * 
 * This function starts a background thread that reads YUV420 frames
 * from the specified file and displays them on the OLED at the
 * appropriate frame rate.
 * 
 * param yuv_filename path to YUV420 video file
 * returns 0 on success, -1 on failure
 *******************************************************************************/
int start_video_display(const char *yuv_filename);

/******************************************************************************
 * Start real-time display.
 * 
 * This function sets up a background thread to read from the named pipe
 * and display frames on the OLED in real-time.
 * 
 * returns 0 on success, -1 on failure
 *******************************************************************************/
 int start_realtime_display(void);

/******************************************************************************
 * Check if a YUV file is currently being displayed.
 * 
 * returns 1 if display is active, 0 if not
 *******************************************************************************/
 int is_display_active(void);

 //UBYTE getData(UBYTE *oled_buffer);

/******************************************************************************
 * Stops video playback.
 * 
 * This function stops the background playback thread and clears
 * the display.
 *******************************************************************************/
 void stop_display(void);

/******************************************************************************
 * Process and display a camera frame on OLED.
 * 
 * Takes a raw YUV420 frame from the camera, converts it to RGB565 format
 * and displays it on the connected OLED display.
 *
 * frame_data Pointer to YUV420 frame data buffer
 * data_size Size of the frame data in bytes
 * 
 * return 0 on success, negative value on error
 *******************************************************************************/
 void display_camera_frame(uint8_t *frame_date, size_t data_size);

 int init_display_buffers(void);
 void free_display_buffers(void);
 
 #endif /* CAM_DRIVER_H */
