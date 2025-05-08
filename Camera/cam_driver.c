/******************************************************************************
* CAM_DRIVER.C
* 
* Implementation of the camera driver for OLED display. This file contains
* the core functionality for controlling an SSD1351-based OLED display with
* real-time camera input or pre-recorded YUV420 video playback.
* 
* The implementation provides thread-based handling of video streams, colorspace
* conversion from YUV420 to RGB565 for display, and proper resource management
* for the OLED hardware. It supports both file-based playback and real-time
* streaming through a named pipe.
*
* Key features:
*   - 30 FPS video playback with timing control
*   - YUV420 to RGB565 colorspace conversion
*   - Threaded handling of display operations
*   - Named pipe support for real-time camera feed
*   - SPI interface to 128x128 OLED display
*
* Hardware requirements:
*   - Waveshare 1.5" RGB OLED display (SSD1327 controller)
*   - GPIO pins for controlling OLED (DC, RST, CS)
*   - SPI interface for data transfer
*
* Dependencies:
*   - lgpio.h - for GPIO control
*   - OLED_1in5_rgb.h - Waveshare OLED driver
*   - pthread.h - POSIX threads
*   - linux/spi/spidev.h - Linux SPI interface
* 
* Author: The One Project is Real!!!!
* Date: 02/19/2025
* License: Version 2.1, February 1999
*
* Copyright (C) 1991, 1999 Free Software Foundation, Inc.
* <https://fsf.org/>
* Everyone is permitted to copy and distribute verbatim copies
* of this license document, but changing it is not allowed.
*
* [This is the first released version of the Lesser GPL.  It also counts
*  as the successor of the GNU Library Public License, version 2, hence
*  the version number 2.1.]
*
 ******************************************************************************/
 #include "cam_driver.h"         // driver header file
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <unistd.h>
 #include <fcntl.h>              // for file control
 #include <errno.h>
 #include <sys/stat.h>       
 #include <signal.h>
 #include <time.h>
 #include <pthread.h>
 #include <sys/ioctl.h>	        // device-specific I/O operations
 #include <sys/types.h>
 #include <linux/spi/spidev.h>   // for SPI device control; transfer data to SPI peripherals
 #include <lgpio.h>              // GPIO control
 #include "OLED_1in5_rgb.h"      // header file for OLED
 #include "GUI_Paint.h"          // header file for OLED colors
 #include "test.h"
 #include "DEV_Config.h"
 
 // OLED Constants for Waveshare 1.5" OLED
 #define OLED_WIDTH  DISPLAY_WIDTH
 #define OLED_HEIGHT DISPLAY_HEIGHT
 #define OLED_DC_PIN 22      // Data/Command pin (BCM numbering)
 #define OLED_RST_PIN 13     // Reset pin
 #define OLED_CS_PIN  24     // Chip select pin
 
 // Control initialization
 static volatile int display_active = 0;
 static volatile int pipe_created = 0;
 static pthread_t display_thread;
 
 // Static function declarations
 static void* display_thread_func(void* arg);
 static void* pipe_thread_func(void* arg);
 //static void yuv420_to_rgb(uint8_t y, uint8_t u, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);
 
 // Allocate OLED buffer (RGB565 format - 2 bytes per pixel)
 //static UWORD *oled_buffer = NULL;
 static UBYTE *oled_buffers[2] = {0, 0}; // double buffering
 static int current_buffer = 0;
 static UWORD buffer_size = (OLED_WIDTH*2) * OLED_HEIGHT;
 //static UBYTE *oled_buffer = NULL;
 
 /******************************************************************************
 * function: oled_init
 * brief: Initialize the OLED display
 * 
 * This function sets up the SPI interface and configures the display
 * for proper operation with the Waveshare 1.5" OLED with SSD1352 controller.
 * It initializes the display module, clears the screen, and allocates memory
 * for the display buffer.
 * 
 * returns 0 on success, -1 on failure
 ******************************************************************************/
 int oled_init(void) 
 {
     printf("Initializing OLED display...\n");
 
     // Initialize the device with driver
     if(DEV_ModuleInit() != 0) 
     {
         printf("DEV_ModuleInit failed\n");
         return -1;
     }
 
     // Initialize the OLED display
     OLED_1in5_rgb_Init();
     DEV_Delay_ms(100);
     
     // Clear the display to start with a blank screen
     OLED_1in5_rgb_Clear();
 
     // Allocate the display buffer just once at initialization
     //buffer_size = (OLED_WIDTH*2) * OLED_HEIGHT; // RGB565 format (2 bytes per pixel)
     //oled_buffer = (UWORD *)malloc(buffer_size);
     // if (oled_buffer == NULL) 
     // {
     //     printf("Failed to allocate display buffer\n");
     //     return -1;
     // }
     if(init_display_buffers() != 0)
     {
         printf("Failed to initialize display buffers\n");
         return -1;
     }
     
     // Initialize buffer to all black
     //memset(oled_buffer, 0, buffer_size);
 
     printf("OLED display initialized\n");
     return 0;
 }
 
 //new code for initializing the buffers at the startup of the program
 int init_display_buffers(void)
 {
     buffer_size = (OLED_WIDTH*2) * OLED_HEIGHT; // RGB565 format (2 bytes per pixel)
 
     if(oled_buffers[0] == NULL)
     {
         oled_buffers[0] = (UBYTE *)malloc(buffer_size);
         if(oled_buffers[0] == NULL)
         {
             perror("Failed to allocate OLED buffer 0");
             return -1;
         }
         memset(oled_buffers[0], 0, buffer_size); // Initialize to black
     }
 
     if(oled_buffers[1] == NULL)
     {
         oled_buffers[1] = (UBYTE *)malloc(buffer_size);
         if(oled_buffers[1] == NULL)
         {
             perror("Failed to allocate OLED buffer 1");
             free(oled_buffers[0]);
             oled_buffers[0] = NULL;
             return -1;
         }
         memset(oled_buffers[1], 0, buffer_size);
     }
 
     return 0;
 }
 
 /******************************************************************************
 * function: yuv420_to_rgb
 * brief: Convert YUV420 color space to RGB
 * 
 * This function implements the standard YUV420 to RGB color conversion formula,
 * with proper clamping of values to ensure valid RGB output in the range [0-255].
 *
 * Parameters:
 *   y - Y component (luminance)
 *   u - U component (chrominance)
 *   v - V component (chrominance)
 *   r - Pointer to store resulting Red component
 *   g - Pointer to store resulting Green component
 *   b - Pointer to store resulting Blue component
 ******************************************************************************/
 void yuv420_to_rgb(uint8_t y, uint8_t u, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) 
 {
     // YUV to RGB conversion formula
     int c = y - 16;
     int d = u - 128;
     int e = v - 128;
     
     // Calculate RGB values
     int r_temp = (298 * c + 409 * e + 128) >> 8;
     int g_temp = (298 * c - 100 * d - 208 * e + 128) >> 8;
     int b_temp = (298 * c + 516 * d + 128) >> 8;
 
     // Calculate RGB components with lookups and fixed-point math
     // int r = (y_val + uvTable[v][3] + 128) >> 8;
     // int g = (y_val + uvTable[u][1] + uvTable[v][4] + 128) >> 8;
     // int b = (y_val + uvTable[u][2] + 128) >> 8;
     
     // Clamp values to valid range [0, 255]
     *r = (r_temp < 0) ? 0 : ((r_temp > 255) ? 255 : r_temp);
     *g = (g_temp < 0) ? 0 : ((g_temp > 255) ? 255 : g_temp);
     *b = (b_temp < 0) ? 0 : ((b_temp > 255) ? 255 : b_temp);
     // int r = (r < 0) ? 0 : ((r > 255) ? 255 : r);
     // int g = (g < 0) ? 0 : ((g > 255) ? 255 : g);
     // int b = (b < 0) ? 0 : ((b > 255) ? 255 : b);
 
     // Convert RGB to RGB565 directly
     //return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
 }
 
 /******************************************************************************
 * function: start_video_display
 * brief: Start video playback on the OLED display
 * 
 * This function starts a background thread that reads YUV420 frames
 * from the specified file and displays them on the OLED at the
 * appropriate frame rate. The video file is played in a loop.
 * 
 * returns 0 on success, -1 on failure 
 *
 * errors: 
 *   - if display is already active
 *   - file doesn't exist
 *   - thread creation fails
 ******************************************************************************/
 int start_video_display(const char *yuv_filename) 
 {
     if (display_active) 
     {
         fprintf(stderr, "Display is active\n");
         return -1;
     }
     
     // Check if file exists
     FILE* test = fopen(yuv_filename, "rb");
     if (!test) 
     {
         perror("Cannot open YUV file");
         return -1;
     }
     fclose(test);
     
     // Set flag and create thread
     display_active = 1;
     
     // Create thread for playback
     if (pthread_create(&display_thread, NULL, display_thread_func, (void*)yuv_filename) != 0) 
     {
         perror("Failed to create display thread");
         display_active = 0;
         return -1;
     }
     
     return 0;
 }
 
 /******************************************************************************
 * function: display_thread_func
 * brief: Thread function for file playback
 * 
 * This function runs in a separate thread and handles reading frames from a YUV420
 * video file, converting them to RGB565, and displaying them on the OLED at the
 * specified frame rate. It implements timing control to maintain constant FPS.
 *
 * returns NULL on completion
 ******************************************************************************/
 static void* display_thread_func(void* arg) 
 {
     const char* filename = (const char*)arg;
     FILE* file = fopen(filename, "rb");
     if (!file) 
     {
         perror("Failed to open YUV file");
         display_active = 0;
         return NULL;
     }
     
     // Allocate frame buffer for YUV data
     size_t frame_size = OLED_WIDTH * OLED_HEIGHT * 3 / 2; // YUV420 size
     uint8_t* frame_buffer = malloc(frame_size);
     if (!frame_buffer) 
     {
         perror("Failed to allocate frame buffer");
         fclose(file);
         display_active = 0;
         return NULL;
     }
 
     // Frame timing variables
     struct timespec frame_time;
     long frame_duration_ns = 1000000000 / FPS;
     
     while (display_active) 
     {
         // Get start time for frame rate control
         clock_gettime(CLOCK_MONOTONIC, &frame_time);
         
         // Read a frame from the file
         size_t bytes_read = fread(frame_buffer, 1, frame_size, file);
         if (bytes_read < frame_size) 
         {
             // End of file or error, rewind to beginning
             rewind(file);
             continue;
         }
         
         // using new function to display frame
         display_camera_frame(frame_buffer, frame_size);
 
         // Calculate time spent processing and delay for consistent FPS
         struct timespec current_time;
         clock_gettime(CLOCK_MONOTONIC, &current_time);
         
         long elapsed_ns = (current_time.tv_sec - frame_time.tv_sec) * 1000000000 +
                           (current_time.tv_nsec - frame_time.tv_nsec);
         
         if (elapsed_ns < frame_duration_ns) 
         {
             // Sleep for remaining time
             struct timespec sleep_time;
             sleep_time.tv_sec = 0;
             sleep_time.tv_nsec = frame_duration_ns - elapsed_ns;
             nanosleep(&sleep_time, NULL);
         }
     }
     
     // Clean up
     free(frame_buffer);
     fclose(file);
     
     return NULL;
 }
 
 /******************************************************************************
 * function: start_realtime_display
 * brief: Start real-time display from a named pipe
 * 
 * This function sets up a background thread to read YUV420 frames from the
 * named pipe and display them on the OLED in real-time. It creates the pipe
 * if it doesn't already exist.
 *
 * returns 0 on success, -1 on failure 
 *
 * errors:
 *   - if display is already active
 *   - pipe creation fails
 *   - thread creation fails
 ******************************************************************************/
 int start_realtime_display(void) 
 {
     if (display_active) 
     {
         fprintf(stderr, "Display already active\n");
         return -1;
     }
     
     // Create pipe if it doesn't exist
     struct stat st;
     if (stat(PIPE_PATH, &st) != 0) 
     {
         if (mkfifo(PIPE_PATH, 0666) != 0) 
         {
             perror("Failed to create FIFO pipe");
             return -1;
         }
         pipe_created = 1;
     }
     
     // Set flag and create thread
     display_active = 1;
     
     // Create thread for pipe reading
     if (pthread_create(&display_thread, NULL, pipe_thread_func, NULL) != 0) 
     {
         perror("Failed to create display thread");
         display_active = 0;
         return -1;
     }
     
     return 0;
 }
 
 /******************************************************************************
 * function: pipe_thread_func
 * brief: Thread function for pipe reading
 *
 * This function runs in a separate thread and handles reading YUV420 frames
 * from the named pipe and displaying them on the OLED. It maintains statistics
 * about frames received and display performance.
 *
 * Parameters:
 *   arg - Unused parameter (required by pthread API)
 *
 * returns NULL on completion
 ******************************************************************************/
 static void* pipe_thread_func(void* arg)
 {
     printf("Pipe thread starting, opening pipe: %s\n", PIPE_PATH);
     int pipe_fd = open(PIPE_PATH, O_RDONLY);
 
     if (pipe_fd < 0)
     {
         perror("Failed to open pipe");
         display_active = 0;
         return NULL;
     }
 
     printf("Pipe opened successfully (fd = %d)\n", pipe_fd);
 
     // Allocate frame buffer for YUV data (using double buffering)
     size_t frame_size = OLED_WIDTH * OLED_HEIGHT * 3 / 2; // YUV420 size
     //uint8_t* frame_buffer = malloc(frame_size);
     uint8_t* frame_buffers[2] = {NULL, NULL}; // two buffers for double buffering
     frame_buffers[0] = malloc(frame_size);
     if (!frame_buffers[0])
     {
         perror("Failed to allocate first frame buffer");
         close(pipe_fd);
         display_active = 0;
         return NULL;
     }
 
     frame_buffers[1] = malloc(frame_size);
     if (!frame_buffers[1])
     {
         perror("Failed to allocate second frame buffer");
         free(frame_buffers[0]);
         close(pipe_fd);
         display_active = 0;
         return NULL;
     }
     
     // if (!frame_buffers[0] || !frame_buffers[1]) 
     // {
     //     perror("Failed to allocate frame buffers");
     //     if(frame_buffers[0])
     //     {
     //         free(frame_buffers[0]);
     //     }
 
     //     if(frame_buffers[1])
     //     {
     //         free(frame_buffers[1]);
     //     }
 
     //     close(pipe_fd);
     //     display_active = 0;
     //     return NULL;
     // }
 
     printf("Double frame buffer allocated, size=%zu bytes\n", frame_size);
     printf("Waiting for data from camera...\n");
 
     int frames_received = 0;
     time_t start_time = time(NULL);
     int active_buffer = 0; // check status of the active buffer
     
     // initially filling the first buffer
     size_t total_read = 0;
     while(total_read < frame_size && display_active)
     {
         ssize_t bytes_read = read(pipe_fd, frame_buffers[0] + total_read, frame_size - total_read);
 
         if(bytes_read <= 0)
         {
             if (bytes_read < 0) 
             {
                 perror("Error reading from pipe");
             } 
             else 
             {
                 printf("Pipe closed by writer\n");
             }
             
             // Break out of the inner loop on error
             break;
         }
 
         total_read += bytes_read;
     }
 
     if (total_read < frame_size) 
     {
         fprintf(stderr, "Incomplete first frame (%zu/%zu bytes)...exiting\n", total_read, frame_size);
         free(frame_buffers[0]);
         free(frame_buffers[1]);
         close(pipe_fd);
         display_active = 0;
         return NULL;
     }
     
     frames_received++;
 
     while (display_active) 
     {
         // read complete frames
         //size_t total_read = 0;
         display_camera_frame(frame_buffers[active_buffer], frame_size);
 
         // While displaying the current buffer, read the next frame into the other buffer
         int next_buffer = 1 - active_buffer; // Switch buffers (0->1, 1->0)
         total_read = 0;
 
         while(total_read < frame_size && display_active)
         {
             ssize_t bytes_read = read(pipe_fd, frame_buffers[next_buffer] + total_read, frame_size - total_read);
 
             if(bytes_read <= 0)
             {
                 if (bytes_read < 0) 
                 {
                     perror("Error reading from pipe");
                 } 
                 else 
                 {
                     printf("Pipe closed by writer\n");
                 }
                 
                 // Break out of the inner loop on error
                 break;
             }
 
             total_read += bytes_read;
         }
         
         // if we did not get a complete frame, exit the thread
         if (total_read < frame_size) 
         {
             fprintf(stderr, "Incomplete frame (%zu/%zu bytes)...exiting\n", total_read, frame_size);
             break;
         }
 
         // using new function to display on OLED
         //display_camera_frame(frame_buffer, frame_size);
         // switch to the newly filled buffer for the next iteration
         active_buffer = next_buffer;
 
         // successfully read a complete frame
         frames_received++;
         if (frames_received % 300 == 0) 
         {  // Log every 300 frames
             time_t now = time(NULL);
             float elapsed = now - start_time;
             if(elapsed > 0)
             {
                 printf("\nReceived %d frames in %.1f seconds (%.2f FPS)\n", 
                     frames_received, elapsed, frames_received / elapsed);
             }
         }
     }
        
     //free(oled_buffer);
     free(frame_buffers[0]);
     free(frame_buffers[1]);
 
     // closing the file descriptor
     if(pipe_fd >= 0)
     {
         close(pipe_fd);
     }
 
     printf("Pipe thread exiting, received %d frames total\n", frames_received);
     
     return NULL;
 }
 
 /******************************************************************************
 * function: display_camera_frame
 * brief: Process and display a camera frame on OLED
 * 
 * Takes a raw YUV420 frame, converts it to RGB565 format and displays it on 
 * the connected OLED display. This function handles the colorspace conversion
 * for each pixel and formats the data properly for the OLED driver.
 *
 *Parameters:
 *   frame_buffer - Pointer to YUV420 frame data buffer
 *   frame_size - Size of the frame data in bytes (must be at least OLED_WIDTH * OLED_HEIGHT * 3/2)
 *
 ******************************************************************************/
 void display_camera_frame(uint8_t* frame_buffer, size_t frame_size) 
 {
     // the buffers are already pre-allocated at the top
     //UBYTE *oled_buffer;
     //UWORD buffer_size = (OLED_WIDTH*2) * OLED_HEIGHT;
 
     if((oled_buffers[0] = (UBYTE *)malloc(buffer_size)) == NULL)
      {
          perror("Failed to allocate OLED buffer");
         return;
      } 
     
     if((oled_buffers[1] = (UBYTE *)malloc(buffer_size)) == NULL)
      {
          perror("Failed to allocate OLED buffer");
         return;
      } 
 
 
     if(oled_buffers[0] == NULL || oled_buffers[1] == NULL)
     {
         fprintf(stderr, "OLED buffer not initialized\n");
         return;
     }
 
     UBYTE *current_buffer1 = oled_buffers[current_buffer]; // use the current buffer for display
 
     // Check if data is valid
     if (!frame_buffer || frame_size < (OLED_WIDTH * OLED_HEIGHT * 3 / 2)) 
     {
         fprintf(stderr, "Invalid frame data or size\n");
         return;
     }
     
     // Process YUV data to RGB565
     uint8_t* y_plane = frame_buffer;
     uint8_t* u_plane = y_plane + OLED_WIDTH * OLED_HEIGHT;
     uint8_t* v_plane = u_plane + (OLED_WIDTH * OLED_HEIGHT / 4);
     
     // Convert YUV to RGB565 & place directly in buffer
     for (int row = 0; row < OLED_HEIGHT; row++) 
     {
         for (int col = 0; col < OLED_WIDTH; col++) 
         {
             int y_index = row * OLED_WIDTH + col;
             int uv_row = row / 2;
             int uv_col = col / 2;
             int uv_index = uv_row * (OLED_WIDTH / 2) + uv_col;
             
             uint8_t y_val = y_plane[y_index];
             uint8_t u_val = u_plane[uv_index];
             uint8_t v_val = v_plane[uv_index];
             
             uint8_t r, g, b;
             yuv420_to_rgb(y_val, u_val, v_val, &r, &g, &b);
             
             // Convert RGB to RGB565 and draw pixel
             UWORD color = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
             UWORD pos = (row * OLED_WIDTH + col) * 2;
             current_buffer1[pos] = (color >> 8) & 0xFF;
             current_buffer1[pos+1] = color & 0xFF;
         }
     }
 
     // Process 2x2 blocks at a time (since U/V are at quarter resolution)
     // for (int row = 0; row < OLED_HEIGHT; row += 2) {
     //     for (int col = 0; col < OLED_WIDTH; col += 2) {
     //         // Get shared U/V for this 2x2 block
     //         int uv_index = (row/2) * (OLED_WIDTH/2) + (col/2);
     //         uint8_t u_val = u_plane[uv_index];
     //         uint8_t v_val = v_plane[uv_index];
             
     //         // Process all 4 pixels in the 2x2 block
     //         for (int y_off = 0; y_off < 2; y_off++) {
     //             for (int x_off = 0; x_off < 2; x_off++) {
     //                 int cur_row = row + y_off;
     //                 int cur_col = col + x_off;
                     
     //                 // Skip if we're outside the display bounds
     //                 if (cur_row >= OLED_HEIGHT || cur_col >= OLED_WIDTH) continue;
                     
     //                 // Get Y value for this specific pixel
     //                 int y_index = cur_row * OLED_WIDTH + cur_col;
     //                 uint8_t y_val = y_plane[y_index];
                     
     //                 // Direct conversion to RGB565
     //                 UWORD color = yuv420_to_rgb(y_val, u_val, v_val);
                     
     //                 // Store in buffer in the format expected by the OLED driver
     //                 int buf_index = y_index;
     //                 oled_buffer[buf_index] = color;
     //             }
     //         }
     //     }
     // }
     
     //return 
     //OLED_1in5_rgb_Display((UBYTE *)oled_buffer);
     OLED_1in5_rgb_Display(current_buffer1);
 
     // switching to the next buffer for the next frame
     current_buffer = 1 - current_buffer;
     
     // Free the buffer
     //free(oled_buffer);
    
     // Display the frame using Waveshare's function
     /*return*/ 
 }
 
 // Add this to free display buffers
 void free_display_buffers(void)
 {
     if(oled_buffers[0] != NULL) 
     {
         free(oled_buffers[0]);
         oled_buffers[0] = NULL;
     }
     
     if(oled_buffers[1] != NULL) 
     {
         free(oled_buffers[1]);
         oled_buffers[1] = NULL;
     }
 }
 
 /******************************************************************************
 * Check if a YUV file is currently being displayed
 * 
 * Returns the current state of the display thread, allowing other parts of
 * the application to determine if video playback is active.
 *
 * Returns:
 *   1 if display is active
 *   0 if display is not active
 ******************************************************************************/
 int is_display_active(void)
 {
     return display_active;
 }
 
 /******************************************************************************
 * Stops video playback
 * 
 * This function stops the background playback thread and sets the
 * display_active flag to 0, which signals the thread to exit.
 * It waits for the thread to complete before returning.
 ******************************************************************************/
 void stop_display(void) 
 {
     if (display_active) 
     {
         display_active = 0;
         pthread_join(display_thread, NULL);
     }
 }
 
 /******************************************************************************
 * Clean up OLED display resources
 * 
 * This function should be called before program exit to properly
 * shut down the display, free hardware resources, and clean up
 * any created pipes. It ensures proper termination of the display thread.
 ******************************************************************************/
 void oled_cleanup(void) 
 {
     if (display_active) 
     {
         stop_display();
     }
 
     // Free the global buffer if it exists
     // if(oled_buffer != NULL) 
     // {
     //     free(oled_buffer);
     //     oled_buffer = NULL;
     // }
 
     // Clean up the Waveshare driver resources
     DEV_ModuleExit();
 
     // Clean up pipe if it was created
     if (pipe_created) 
     {
         unlink(PIPE_PATH);
         pipe_created = 0;
     }
 }
 
 
 