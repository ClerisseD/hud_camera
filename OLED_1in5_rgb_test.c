/*****************************************************************************
* | File      	:   OLED_1in5_rgb_test.c
* | Author      :   Waveshare team
* | Function    :   1.5inch OLED Module test demo
* | Info        :
*----------------
* |	This version:   V2.0
* | Date        :   2020-08-17
* | Info        :
* -----------------------------------------------------------------------------
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "test.h"
#include "OLED_1in5_rgb.h"
#include <stdio.h>
#include <string.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

#include <sys/time.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <sys/stat.h>
#include <pthread.h>
#include "cam_driver.h"

// Define the serial port for UART communication
#define SERIAL_PORT "/dev/serial0"
#define BUTTON_PIN 2
#define CHIP 0
#define BUFFER_SIZE 1024
#define MAX_ENTRIES 1000  // Adjust based on expected data size
#define MAX_CELL_SIZE 100

// Shared battery percentage value and mutex
float latest_battery_percentage = -1.0;
pthread_mutex_t battery_mutex = PTHREAD_MUTEX_INITIALIZER;

volatile int run = 1; // Must be global and volatile for interrupt safety
volatile int state;
volatile int e;
volatile int count = 0;
volatile int count2 = 0;


typedef struct {
    char step_ID[MAX_CELL_SIZE];
    char routeState[MAX_CELL_SIZE];
    char street_name[MAX_CELL_SIZE];
    char distance[MAX_CELL_SIZE];
    char maneuverID[MAX_CELL_SIZE];
    char arrival_time[MAX_CELL_SIZE];
    char currentTemp[MAX_CELL_SIZE];
    char currentWeather[MAX_CELL_SIZE];
    char instructions[MAX_CELL_SIZE];
} CSVData;

static CSVData data = {0}; // 

typedef enum {
	IDLE,
	PRESSED,
	LOCKED
} ButtonState;

// Button LOCK Code 
int Button_Lock(int handle, int pin)
{
	int num = 0;
	int check = 0;
	int temp;
	int check2 = 0;
	ButtonState state = IDLE;
	switch (state)
	{
		case IDLE:
			if (lgGpioRead(handle, pin) == 1)
			{
				state = LOCKED;
				check = 1;
				printf("Going to LOCKED\n");
			}
			else 
			{
				state = IDLE;
			}
		case LOCKED:
			if (check == 1)
			{
				temp = lgGpioRead(handle, pin);
			}
			while (temp == 1)
			{
				state = LOCKED;
				check2 = 1;
				temp = lgGpioRead(handle, pin);
			}
			if (check2 == 1)
			{
				printf("Going to Idle\n");
				state = IDLE;
				num = 1;
				return num;
			}	
	}
	lgGpioFree(handle, pin);
}

// New version of debounce code
int debounceButton(int handle, int pin) 
{
    int lastState = lgGpioRead(handle, pin);
    int count = 0;

    // Ensure the button state remains stable for at least 10ms (10 checks at 1ms intervals)
    for (int i = 0; i < 10; i++)
	 {
        usleep(1000);  // 1ms delay
        if (lgGpioRead(handle, pin) == lastState) 
		{
            count++;
        } else
		{
            count = 0;
            lastState = lgGpioRead(handle, pin);  // Reset check if state changes
        }
    }

    // Return the stable state only if it's consistent
    return (count >= 10) ? lastState : -1;
}


float calculate_battery_percentage(float voltage) 
{
    const float min_voltage = 3.0;  // Minimum voltage corresponding to 0%
    const float max_voltage = 3.8;  // Maximum observed voltage corresponding to 100%
    
    if (voltage <= min_voltage) return 0.0;
    if (voltage >= max_voltage) return 100.0;
    
    return ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0;
}

// Function to remove newline and carriage return from UART input 
void strip_newline(char *str)
{
	char *pos;
	if ((pos = strchr(str, '\n')) != NULL) *pos = '\0';
	if ((pos = strchr(str, '\r')) != NULL) *pos = '\0';
}

// Function to display the directions to the OLED of where to go
void displayImage(char *input) 
{
	char filepath[50] = "./pic/";
	// Map the input string to the corresponding BMP file
	if (strcmp(input, "0") == 0)
	{
		strcat(filepath, "Left_arrow_0_2.0.bmp");
		// Display the corresponding image
		printf("Displaying: %s\n", filepath); // Debugging output
		GUI_ReadBmp(filepath, 25, 50);
	}
	else if (strcmp(input, "1") == 0)
	{
		strcat(filepath, "Right_arrow_1.bmp");
		// Display the corresponding image
		printf("Displaying: %s\n", filepath); // Debugging output
		GUI_ReadBmp(filepath, 15, 50);
	}
	else if (strcmp(input, "2") == 0)
	{
		strcat(filepath, "Straight_arrow_2.bmp");
		// Display the corresponding image
		printf("Displaying: %s\n", filepath); // Debugging output
		GUI_ReadBmp(filepath, 40, 50);
	}
	else if (strcmp(input, "3") == 0)
	{
		strcat(filepath, "U_turn_3.bmp");
		// Display the corresponding image
		printf("Displaying: %s\n", filepath); // Debugging output
		GUI_ReadBmp(filepath, 20, 50);
	}
	else if (strcmp(input, "4") == 0)
	{
		strcat(filepath, "Roundabout_4.bmp");
		// Display the corresponding image
		printf("Displaying: %s\n", filepath); // Debugging output
		GUI_ReadBmp(filepath, 30, 50);
	}
	else if (strcmp(input, "5") == 0)
	{
		strcat(filepath, "Destination_5.bmp");
		// Display the corresponding image
		printf("Displaying: %s\n", filepath); // Debugging output
		GUI_ReadBmp(filepath, 40, 50);
	}
	else if (strcmp(input, "6") == 0)
	{
		strcat(filepath, "Merge_6.bmp");
		// Display the corresponding image
		printf("Displaying: %s\n", filepath); // Debugging output
		GUI_ReadBmp(filepath, 40, 50);
	}
	else if (strcmp(input, "-1") == 0)
	{
		strcat(filepath, "Question_-1.bmp");
		// Display the corresponding image
		printf("Displaying: %s\n", filepath); // Debugging output
		GUI_ReadBmp(filepath, 30, 55);
	}
	else
       	{
		printf("Second :%s\n", input);
		printf("Invalid input: No matching image found.\n");
		return;
	}
}

CSVData last_valid_data = {"N/A", "N/A", "N/A", "N/A", "N/A", "N/A", "N/A", "N/A", "N/A"};

// Function to parse CSV data into a 1D array
int parse_csv(const char *filename, CSVData *data) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Error opening CSV file");
        return -1;
    }

    char line[BUFFER_SIZE];
    if (fgets(line, sizeof(line), file) == NULL) { 
        printf("Warning: CSV file is empty or missing header. Using last valid data.\n");
        *data = last_valid_data; // Use last valid data
        fclose(file);
        return 0;
    }

    if (fgets(line, sizeof(line), file)) { // Read data line
        char *token = strtok(line, ",");
        if (token) strncpy(data->step_ID, token, MAX_CELL_SIZE - 1);
        	
        token = strtok(NULL, ",");
        if (token) strncpy(data->routeState, token, MAX_CELL_SIZE - 1);      
                
        token = strtok(NULL, ",");
        if (token) strncpy(data->street_name, token, MAX_CELL_SIZE - 1);
        
        token = strtok(NULL, ",");
        if (token) strncpy(data->distance, token, MAX_CELL_SIZE - 1);
        
        token = strtok(NULL, ",");
        if (token) strncpy(data->maneuverID, token, MAX_CELL_SIZE - 1);
        
        token = strtok(NULL, ",");
        if (token) strncpy(data->arrival_time, token, MAX_CELL_SIZE - 1);
        
        token = strtok(NULL, ",");
        if (token) strncpy(data->currentTemp, token, MAX_CELL_SIZE - 1);

	token = strtok(NULL, ",");
        if (token) strncpy(data->currentWeather, token, MAX_CELL_SIZE - 1);

	token = strtok(NULL, ",");
        if (token) strncpy(data->instructions, token, MAX_CELL_SIZE - 1);
                
        last_valid_data = *data; // Save last valid data
    } else {
        printf("Warning: CSV file contains no valid data. Using last valid data.\n");
        *data = last_valid_data; // Use last valid data
    }

    fclose(file);
    return 0; // Return the number of values parsed
}

// Bluetooth receiver function
void *bluetooth_receiver(void *arg) 
{
    (void)arg;
    struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
    char buf[BUFFER_SIZE] = { 0 };
    int server_socket, client, bytes_read;
    socklen_t opt = sizeof(rem_addr);

    // Create Bluetooth socket
    server_socket = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if (server_socket < 0) 
    {
        perror("Socket creation failed");
        pthread_exit(NULL);
    }

    // Bind socket to Bluetooth RFCOMM channel 1
    loc_addr.rc_family = AF_BLUETOOTH;
    loc_addr.rc_bdaddr = *BDADDR_ANY;
    loc_addr.rc_channel = (uint8_t) 1;

    if (bind(server_socket, (struct sockaddr *)&loc_addr, sizeof(loc_addr)) < 0) 
    {
        perror("Bind failed");
        close(server_socket);
        pthread_exit(NULL);
    }

    // Listen for incoming Bluetooth connections
    if (listen(server_socket, 1) < 0) 
    {
        perror("Listen failed");
        close(server_socket);
        pthread_exit(NULL);
    }

    printf("Bluetooth Receiver: Waiting for incoming Bluetooth connection...\n");

    while (1) 
    {
        // Accept a client connection
        client = accept(server_socket, (struct sockaddr *)&rem_addr, &opt);
        if (client < 0) 
        {
            perror("Client connection failed");
            continue; // Keep waiting for new connections
        }

        // Print the Bluetooth address of the connected client
        char client_addr[18] = { 0 };
        ba2str(&rem_addr.rc_bdaddr, client_addr);
        printf("Bluetooth Receiver: Accepted connection from %s\n", client_addr);

        // Write battery CSV
        FILE *battery_file = fopen("battery.csv", "w");
        if (!battery_file) 
        {
            perror("Battery file open failed");
        } 
        
        else 
        {
            pthread_mutex_lock(&battery_mutex);
            fprintf(battery_file, "Battery Percentage\n%.2f\n", latest_battery_percentage);
            pthread_mutex_unlock(&battery_mutex);
            fclose(battery_file);
            printf("Battery CSV created.\n");
        }

        // Send battery CSV
        battery_file = fopen("battery.csv", "r");
        if (!battery_file) 
        {
            perror("Battery file open for sending failed");
        } 
        
        else 
        {
            char send_buf[BUFFER_SIZE];
            size_t read_bytes;
            printf("Sending battery CSV to client...\n");

            while ((read_bytes = fread(send_buf, 1, sizeof(send_buf), battery_file)) > 0) 
            {
                if (send(client, send_buf, read_bytes, 0) < 0) 
                {
                    perror("Failed to send battery file");
                    break;
                }
            }

            fclose(battery_file);
            printf("Battery CSV sent successfully.\n");
        }

        // Open file to store received CSV data
        FILE *file = fopen("received.csv", "w");
        if (!file) 
        {
            perror("File open failed");
            close(client);
            continue; // Wait for a new connection
        }

        printf("Bluetooth Receiver: Receiving CSV file...\n");

        // Receive data and write to file
        while ((bytes_read = recv(client, buf, BUFFER_SIZE, 0)) > 0) 
        {
            fwrite(buf, 1, bytes_read, file);
        }

        fclose(file); // Close file after receiving data

        // Parse received CSV data
        CSVData data = {0};

        close(client); // Disconnect client after sending battery data
    }

    close(server_socket); // Close the server socket when done
    pthread_exit(NULL);
    return NULL;
}


//Function for Interrupt code
void button_callback(int num, lgGpioAlert_p evt, void *user)
{
	if (evt[num].report.level == 0)
	{ 
		// Button pressed (assuming active-low)
		printf("Interrupt: Button pressed! Exiting GPS state.\n");
		run = 0;
		e = 0;
	}

}

// Function to continuously read battery voltage from UART input
void read_battery_voltage() 
{
    FILE *uart_input = fopen(SERIAL_PORT, "r"); // Open the UART serial port for reading

    // Check if the UART port was opened successfully
    if (!uart_input) 
    {
        perror("Failed to open UART"); // Print error message if the port cannot be opened
        return;
    }

    printf("UART communication established. Listening for messages...\n");

    char received_line[256]; // Buffer to store received UART data

    // Infinite loop to continuously read battery voltage data
    while (1) 
    {
        // Read a line of data from UART input
        if (fgets(received_line, sizeof(received_line), uart_input)) 
        {
            strip_newline(received_line);  // Remove newline characters from the received string
            
            float voltage = 0.0;
            
            // Extract voltage value from received string using sscanf
            if (sscanf(received_line, "Battery Voltage: %f V", &voltage) == 1) 
            {
                float percentage = calculate_battery_percentage(voltage); // Convert voltage to percentage

                pthread_mutex_lock(&battery_mutex);
                latest_battery_percentage = percentage;
                pthread_mutex_unlock(&battery_mutex);

                // Print the received voltage and calculated battery percentage
                printf("\nReceived: Battery Voltage: %.2f V\n", voltage);
                printf("Battery Percentage: %.2f%%\n", latest_battery_percentage);

                // Display warning if voltage drops below 3.3V
                if (voltage < 3.3) 
                {
                    printf("Warning: Low Battery! Voltage below 3.3V.\n");
                }
            }
        }

        usleep(100000); // Sleep for 100 milliseconds to reduce CPU usage
    }

    fclose(uart_input); // Close the UART input file (this code is unreachable due to infinite loop)
}


int OLED_1in5_rgb_test(void)
{
	
	// Setting up the thread for Bluetooth
	pthread_t bt_thread;
	pthread_t battery_thread;
	// Error Checking for Bluetooth
	if (pthread_create(&bt_thread, NULL, bluetooth_receiver, NULL) != 0)
	{
		perror("Failed to create bluetooth receiver thread");
		return 1;
	}
	
	
	if (pthread_create(&battery_thread, NULL, (void *)read_battery_voltage, NULL) != 0) 
    {
        perror("Failed to create battery voltage reader thread");
        return 1;
    }
	
	// Setting up the Button GPIO pins
	int h = lgGpiochipOpen(CHIP);

	if(h < 0)
	{
		perror("Failed to open GPIO chip");
		return 1;
	}
	// Make sure it's a to active low input 
	if (lgGpioClaimInput(h, LG_SET_PULL_UP, BUTTON_PIN) < 0) 
	{
		perror("Failed to claim input GPIO");
		lgGpiochipClose(h);
		return 1;
	}
	
	lgGpioFree(h, BUTTON_PIN);
	printf("Press the button (GPIO %d)...\n", BUTTON_PIN);
	

	// Setting up the OLED display
	printf("1.5inch RGB OLED test demo\n");
	if(DEV_ModuleInit() != 0) {
		return -1;
	}

	if(USE_IIC) {
		printf("Only USE_SPI, Please revise DEV_Config.h !!!\r\n");
		return -1;
	}

	// Initialise the OLED display 
	printf("OLED Init...\r\n");
	OLED_1in5_rgb_Init();
	DEV_Delay_ms(500);	
	// 0.Create a new image cache
	static UBYTE *BlackImage;
	static UWORD Imagesize = (OLED_1in5_RGB_WIDTH*2) * OLED_1in5_RGB_HEIGHT;
	if((BlackImage = (UBYTE *)malloc(Imagesize + 300)) == NULL) {
		printf("Failed to apply for black memory...\r\n");
		return -1;
	}
	printf("Paint_NewImage\r\n");
	Paint_NewImage(BlackImage, OLED_1in5_RGB_WIDTH, OLED_1in5_RGB_HEIGHT, 0, BLACK);	// Creates the new Image cache 
	Paint_SetScale(65);
	printf("Drawing\r\n");
	//1.Select Image
	Paint_SelectImage(BlackImage);
	DEV_Delay_ms(500);
	Paint_Clear(BLACK);
	OLED_1in5_rgb_Display(BlackImage);

	int state = 1;
	static int userdata = 123;

	lgGpioSetDebounce(h, BUTTON_PIN, 195000);

	// Change to this code so that it's more organized
	while (1) 
	{
		// State handling
		switch (state) 
		{
			/*
			case 2:// Spotify
			       while(1)
			       {	/       
					int num2;
					num2 = lgGpioRead(h, BUTTON_PIN);
					printf("Num: %d\n", num2);

					if (num2 == 1)
					{
						state = 1;
						break;
					}/


			       }
			      */

			case 1: // GPS State
				while(run)
				{	
					int num2; 
					num2 = Button_Lock(h, BUTTON_PIN);
					if (num2 == 1)
					{
						state = 0;
						run = 0;
						break;
					}							
							parse_csv("received.csv", &data); // Parses the data and stores it into a struct 
							
							// If not arrived displays the GPS, Weather, Battery 
							
							if (strcmp(data.routeState,"ARRIVED") != 0)
							{
								displayImage(data.maneuverID); //Directions 

								Paint_DrawString_EN(0, 40, data.street_name, &Font12, BLACK, WHITE); //Street name 

								Paint_DrawString_EN(0, 115, data.arrival_time , &Font12, BLACK, WHITE); //Arrival Time 

								Paint_DrawString_EN(70, 50, data.distance, &Font12, BLACK, WHITE);// Distance 

								Paint_DrawNum(0, 50, latest_battery_percentage, &Font12, 2 , WHITE, BLACK); //Battery 
								Paint_DrawString_EN(35, 50, "%", &Font12, BLACK, WHITE);

								Paint_DrawString_EN(100, 115, data.currentTemp, &Font12, BLACK, WHITE); //Temp

								char Weather[100];
								strncpy(Weather, data.currentWeather, 100);
								printf("Current weather: %s\n", Weather);
								Paint_DrawString_EN(55, 115, Weather, &Font12, BLACK, WHITE); // Current Weather
								printf("Route State: %s\n" ,data.routeState); 

								// Function for returning the display onto the actually OLED
								OLED_1in5_rgb_Display(BlackImage);
								Paint_Clear(BLACK);
							}	
							else 
							{
								// Displays when the user has arrived at there Destintation 
								Paint_DrawString_EN(0, 40, "YOU HAVE ARRIVED!", &Font12, BLACK, WHITE); //Street name 
								displayImage(data.maneuverID); // Displays Direction arrows
								// Function for returning the display onto the actually OLED
								OLED_1in5_rgb_Display(BlackImage);
								Paint_Clear(BLACK);
							}
							Paint_Clear(BLACK);
							
				}
				// Frees up the data for the button 
				lgGpioFree(h, BUTTON_PIN);
				state = 0;
				printf("Exit while loop\n");
				break;
			case 0: // Camera State
				printf("Displaying Camera State\n");
				// Create the pipe if it doesn't exist
				struct stat st;
				if (stat(PIPE_PATH, &st) != 0) 
				{
					if (mkfifo(PIPE_PATH, 0666) != 0) 
					{
						perror("Failed to create FIFO pipe");
						Paint_DrawString_EN(10, 50, "Pipe Error!", &Font12, BLACK, RED);
						OLED_1in5_rgb_Display(BlackImage);
						sleep(2);
						break;
					}
					printf("Created named pipe: %s\n", PIPE_PATH);
				}
				
				// Start real-time display thread first
				if (!is_display_active()) 
				{
					printf("Starting display thread...\n");
					if (start_realtime_display() != 0) 
					{
						printf("Failed to start display thread\n");
						Paint_DrawString_EN(10, 50, "Display Error!", &Font12, BLACK, RED);
						OLED_1in5_rgb_Display(BlackImage);
						sleep(2);
						break;
					}
				}
				
				// Now start the camera with output to the pipe
				printf("Starting camera...\n");
				
				// Kill any existing camera processes
				system("pkill -f libcamera-vid");
				
				// Start a new camera process
				char camera_cmd[1024];
				snprintf(camera_cmd, sizeof(camera_cmd),
						"libcamera-vid "
						"--vflip "
						"--width %d --height %d "
						"--framerate %d "
						"--codec yuv420 "
						"--timeout 300000 "  // 5 minutes recording timeout
						"--output - > %s &", // Direct to the pipe
						DISPLAY_WIDTH, DISPLAY_HEIGHT, FPS, PIPE_PATH);
				
				printf("Executing: %s\n", camera_cmd);
				system(camera_cmd);
				
				// Give the camera time to start
				sleep(1);
				// Wait for button press or timeout
				
				time_t start_time = time(NULL);
				//printf("Camera running, press the button to exit...\n");
				while (state == 0 && (time(NULL) - start_time) < 300) 
				{	
					int num1; 
					num1 = Button_Lock(h, BUTTON_PIN);
					if (num1 == 1)
					{
						state = 1;
						run = 1;
						break;
					}
					usleep(100000);  // 100ms delay
				}
				// Clean up camera
				printf("Stopping camera...\n");
				system("pkill -f libcamera-vid"); 
				
				// Stop display thread
				if (is_display_active()) 
				{
					printf("Stopping display thread...\n");
					stop_display();
				}
				
				lgGpioFree(h, BUTTON_PIN);
			
		}
		usleep(100000);  // 100ms delay to control CPU usage
	}
	// Closes the UART, lgGPIO, Pthread 
	lgGpioFree(h, BUTTON_PIN);
	lgGpiochipClose(h);
	pthread_join(bt_thread, NULL);
	pthread_join(battery_thread, NULL);

	return 0;
}
