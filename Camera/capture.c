/***************************************************************************
* filename: capture.c
* brief: Camera capture and display system for Raspberry Pi
*        camera module 3
*
* This program implements a capture system that records video in
* both raw YUV and compressed H264 formats. It includes real-time
* display capabilities on the Waveshare 1.5" OLED screen and supports
* video playback. The system has resolution and framerate that can be
* set to any value while saving with dated and timestamped filenames.
* The system handles proper cleanup on program termination via signal
* handling.
*
* author: The One Project is Real!!!!
* date: 02/05/2025
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
****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>          // for file control
#include <errno.h>          // for error handling
#include <signal.h>         // for handling interrupt signals
#include "cam_driver.h"     // include the OLED driver header

#define FPS 12                                  // capturing at 30 frames per second
#define DIR_OUTPUT "captured_videos"
#define MAX_CMD_LENGTH 1024
#define MAX_FILENAME_LENGTH 256
#define DURATION_MS 300000                       // record for 5 minutes

volatile sig_atomic_t keep_running = 1;
static volatile int pipe_created = 0;
char current_yuv_file[MAX_FILENAME_LENGTH] = {0};  // global variable

/***************************************************************************
* function: signal_handler
* brief: Signal handler for interrupt signals
*
* Handles SIGINT (Ctrl+C) by setting the keep_running flag to false and
* stopping any active display process for clean program termination.
*
****************************************************************************/
void signal_handler(int signum) 
{
    if (signum == SIGINT) 
    {
        printf("\nCaught interrupt signal. Cleaning up...\n");
        keep_running = 0;

        // stop any active display when exiting
        if (is_display_active())
        {
            stop_display();
        }
    }
}

/***************************************************************************
* function: create_directory
* brief: Creates a directory if it does not exist
*
* Checks if a specified directory exists and creates it with 0777
* permissions if it does not exist yet.
*
* returns 0 on success, -1 on error
****************************************************************************/
int create_directory(const char *dir) 
{
    struct stat st = {0};
    if (stat(dir, &st) == -1) 
    {
        if (mkdir(dir, 0777) < 0) 
        {
            perror("Failed to create directory");
            return -1;
        }
    }
    return 0;
}

/***************************************************************************
* function: filename_gen
* brief: Generates a dated and timestamped filename for video output.
*
* Creates a filename with the current date and time in the format:
* DIR_OUTPUT/video_MMDDYYYY_HHMMSS.format (YUV or H264)
*
****************************************************************************/
void filename_gen(char *buffer, size_t size, const char *format) 
{
    if (!buffer || size < 1) return;
    
    time_t now = time(NULL);
    if (now == -1) 
    {
        fprintf(stderr, "Failed to get current time\n");
        return;
    }

    struct tm *t = localtime(&now);
    if (!t) 
    {
        fprintf(stderr, "Failed to convert time\n");
        return;
    }

    snprintf(buffer, size, "%s/video_%02d%02d%04d_%02d%02d%02d.%s",
            DIR_OUTPUT,
            t->tm_mon + 1,
            t->tm_mday,
            t->tm_year + 1900,
            t->tm_hour,
            t->tm_min,
            t->tm_sec,
            format);
}

/***************************************************************************
* function: setup_display_env
* brief: Sets up the display environment for X11
*
* Configures the DISPLAY environment variable and grants access to the X
* server for local connections.
*
****************************************************************************/
void setup_display_env() // Ensure proper display access
{
    setenv("DISPLAY", ":0", 1);
    system("xhost +local: >/dev/null 2>&1");    
}

/***************************************************************************
* function: capture_video
* brief: Captures video in both YUV and H264 formats
*
* Uses libcamera library commands to capture video in both YUV and H264
* formats simultaneously. If realtime_display is enabled, it streams the
* YUV data to a named pipe for real-time display on the OLED screen.
*
* returns 0 on success, -1 on any error code failure
*
****************************************************************************/
int capture_video(const char *yuv_filename, const char *h264_filename, int realtime_display) 
{
    char command [MAX_CMD_LENGTH];

    // Check if output files can be created
    int yuv_fd = open(yuv_filename, O_WRONLY | O_CREAT | O_EXCL, 0644);
    if (yuv_fd < 0 && errno != EEXIST) 
    {
        fprintf(stderr, "Cannot create YUV output file '%s': %s\n", yuv_filename, strerror(errno));
        return -errno;
    }
    else if (yuv_fd >= 0) 
    {
        close(yuv_fd);
    }
    
    int h264_fd = open(h264_filename, O_WRONLY | O_CREAT | O_EXCL, 0644);
    if (h264_fd < 0 && errno != EEXIST) 
    {
        fprintf(stderr, "Cannot create H264 output file '%s': %s\n", h264_filename, strerror(errno));
        return -errno;
    }
    else if (h264_fd >= 0) 
    {
        close(h264_fd);
    }

    // setting up named pipe for the real-time display
    if (realtime_display)
    {
        // Make sure pipe exists and has correct permissions
        struct stat st;
        if (stat(PIPE_PATH, &st) != 0) 
        {
            // Create the pipe if it doesn't exist
            if (mkfifo(PIPE_PATH, 0666) != 0) 
            {
                perror("Failed to create FIFO pipe");
                return -1;
            }
            pipe_created = 1;
        } 
        else if (!S_ISFIFO(st.st_mode)) 
        {
            // Path exists but is not a pipe
            fprintf(stderr, "%s exists but is not a pipe\n", PIPE_PATH);
            return -1;
        }
        
        // Make sure permissions are correct
        chmod(PIPE_PATH, 0666);

        //starting the real-time display before starting the capturing
        if (start_realtime_display() != 0)
        {
            fprintf(stderr, "Failed to start real-time display\n");
            return -1;
        }

        // small delay to make sure thread is ready
        //sleep(1);

        // commands to capture both YUV and H264 formats
        snprintf(command, sizeof(command),
                "libcamera-vid "                                // libcamera command
                "--width %d --height %d "                       // resolution for the OLED
                "--framerate %d "                               // set to 30 FPS
                "--codec yuv420 "                               // specify YUV format
                "--timeout %d "                                 // records for 5 minutes
                "--output - | tee '%s' > %s "                   // where to save the file (both file and pipe)
                "& " 
                "libcamera-vid "                                // libcamera command
                "--width %d --height %d "                       // resolution for the OLED
                "--framerate %d "                               // set to 30 FPS
                "--codec h264 "                                 // specify h264 format
                "--timeout %d "                                 // records for 5 minutes
                "--output '%s' ",                                // where to save the file
                DISPLAY_WIDTH, DISPLAY_HEIGHT, FPS, DURATION_MS, yuv_filename, PIPE_PATH,
                DISPLAY_WIDTH, DISPLAY_HEIGHT, FPS, DURATION_MS, h264_filename);

        printf("Executing: %s\n", command);
        int pid = fork();
        if(pid == 0)
        {
            // child process
            execl("/bin/sh", "sh", "-c", command, NULL);

            // exec failed
            perror("Failed to execute command");
            return -1;
        }

        // parent process continues here
        if(pid < 0)
        {
            perror("Failed to fork for camera process");
            return -1;
        }

        // store PID for future reference if we really need it
        printf("Camera process started with PID %d\n", pid);
    }
    else
    {
        // standard capture without real-time display
        snprintf(command, sizeof(command),
                "libcamera-vid "                                // libcamera command
                "--width %d --height %d "                       // resolution for the OLED
                "--framerate %d "                               // set to 30 FPS
                "--codec yuv420 "                               // specify YUV format
                "--timeout %d "                                 // records for 30 seconds
                "--output '%s' "                                // where to save the file
                "& " 
                "libcamera-vid "                                // libcamera command
                "--width %d --height %d "                       // resolution for the OLED
                "--framerate %d "                               // set to 30 FPS
                "--codec h264 "                                 // specify h264 format
                "--timeout %d "                                // records for 30 seconds
                "--output '%s' ",                                // where to save the file
                DISPLAY_WIDTH, DISPLAY_HEIGHT, FPS, DURATION_MS, yuv_filename,
                DISPLAY_WIDTH, DISPLAY_HEIGHT, FPS, DURATION_MS, h264_filename);

        printf("Executing command: %s\n", command);
        int result = system(command);
        if (result != 0)
        {
            fprintf(stderr, "Command failed\n");
            return -1;
        }
    }
    
    // If using real-time display, wait for camera to finish,
    // then stop the display
    if (realtime_display && is_display_active()) 
    {
        // Wait for the user to press Ctrl+C instead of completion
        printf("Continuous capture active...\n");
        while (keep_running)
        {
            sleep(1); // sleep to reduce CPU usage
        }
        //sleep(DURATION_MS / 1000 + 1);  // Convert to seconds and add 1 for safety
        
        printf("Stopping real-time display...\n");
        stop_display();
    }
    
    return 0;
}

/***************************************************************************
* function: play_latest_video
* brief: Plays back the most recently captured video
*
* Checks if a recent video file is available and plays it back on the 
* OLED display. It first stops any active display before starting playback.
*
****************************************************************************/
void play_latest_video() // Function to handle playback of the most recently captured video
{
    if (strlen(current_yuv_file) > 0) 
    {
        printf("Playing back most recent capture: %s\n", current_yuv_file);
        
        // Stop any active display first
        if (is_display_active()) 
        {
            stop_display();
        }
        
        // Start playback of the YUV file
        if (start_video_display(current_yuv_file) != 0) 
        {
            fprintf(stderr, "Failed to start video playback\n");
        }
        else
        {
            printf("Video playback started successfully\n");
        }
    } 
    else 
    {
        printf("No video file available for playback\n");
    }
}

/***************************************************************************
* function: main_two
* brief: Main function for the camera capture program.
*
* Initializes the system, sets up signal handling, initializes the OLED 
* display, creates the output directory, and enters the main capture loop. 
* In each iteration, it captures video, displays it in real-time, and can 
* optionally play back the captured video afterward.
*
* returns 0 on success, non-zero value on failure
****************************************************************************/
int main_two(void) 
{
    // Set up signal handling
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sigaction(SIGINT, &sa, NULL);

    // Initialize the OLED display
    printf("Initializing OLED display...\n");
    if (oled_init() != 0)
    {
        fprintf(stderr, "Failed to initialize OLED display\n");
        return 1;
    }

    // Create output directory
    if (create_directory(DIR_OUTPUT) != 0) 
    {
        oled_cleanup();
        return 1;
    }

    //set up display environment
    setup_display_env();

    printf("Starting rear-view camera (%dx%d)...\n", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    printf("OLED display initialized...\n");
    printf("Press Ctrl+C to stop\n");

    // Main capture loop
    int use_realtime_display = 1;  // Set to 1 to enable real-time display during capture

    while (keep_running) 
    {
        //generate filename
        char yuv_filename[MAX_FILENAME_LENGTH];         // for the raw video file
        char h264_filename[MAX_FILENAME_LENGTH];        // for the compressed video file
        
        filename_gen(yuv_filename, sizeof(yuv_filename), "yuv420");
        filename_gen(h264_filename, sizeof(h264_filename), "h264");

        // Save current YUV filename for playback
        strncpy(current_yuv_file, yuv_filename, MAX_FILENAME_LENGTH - 1);
        current_yuv_file[MAX_FILENAME_LENGTH - 1] = '\0';  // Ensure null-termination

        printf("Starting capture:\n");
        printf("YUV file: %s\n", yuv_filename);
        printf("H264 file: %s\n", h264_filename);

        // Capture video with optional real-time display
        int result = capture_video(yuv_filename, h264_filename, use_realtime_display);
        
        if (result != 0) 
        {
            fprintf(stderr, "Video capture failed with code: %d\n", result);
            break;
        }

        printf("Files saved:\n");
        printf("- Raw YUV: %s\n", yuv_filename);
        printf("- H264: %s\n\n", h264_filename);

        // If using real-time display, stop it after capture
        if (use_realtime_display && is_display_active()) 
        {
            stop_display();
        }

        // After capture, play back the file on the OLED
        if (!use_realtime_display) 
        {
            play_latest_video();

            // Give some time to view the playback
            printf("Playing video for 10 seconds...\n");
            sleep(10);
            
            // Stop playback
            if (is_display_active()) 
            {
                stop_display();
            }
        }

        //wait before next capture
        printf("Waiting 5 seconds before next capture...\n");
        sleep(5);
    }

    // Clean up resources
    if (is_display_active()) 
    {
        stop_display();
    }

    //oled_cleanup();
    //
    
    if (pipe_created)
    {
    	unlink(PIPE_PATH);
	pipe_created = 0;
    }
    
    printf("Capture complete. Exiting..\n");
    return 0;
}
