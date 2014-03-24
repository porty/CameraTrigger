// blink.c
//
// Example program for bcm2835 library
// Blinks a pin on an off every 0.5 secs
//
// After installing bcm2835, you can build this 
// with something like:
// gcc -o blink blink.c -l bcm2835
// sudo ./blink
//
// Or you can test it before installing with:
// gcc -o blink -I ../../src ../../src/bcm2835.c blink.c
// sudo ./blink
//
// Author: Mike McCauley
// Copyright (C) 2011 Mike McCauley
// $Id: RF22.h,v 1.21 2012/05/30 01:51:25 mikem Exp $

#include <bcm2835.h>

#include <signal.h>
#include <iostream>
#include <sys/wait.h>

//#include <cstdio>
#include <stdio.h>   // fprintf()
#include <unistd.h>  // fork(), exec()
#include <string.h>  // strerror()
#include <errno.h>   // errno
#include <stdlib.h>  // exit()
#include <time.h>
#include <sys/resource.h> // PRIO_PROCESS

#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

#include <mavlink.h>

// Input on RPi pin GPIO 15
#define PIN RPI_GPIO_P1_15
//states
#define WaitForTrigger 1
#define WaitForData 2
#define WaitForPicture 3
#define PictureFailCleanup 4

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

int uart0_filestream = -1;
int forkAndExecute (const char *path, const char *const args[]) ;

const char* picName = "temp.jpg";
const char* txtName = "temp.txt";


bool handleMessage(mavlink_message_t* msg)
{
    //struct Location tell_command = {};                // command for telemetry

    if (msg->msgid == MAVLINK_MSG_ID_CAMERA_DATA)
	{
		char charBuffer[256];
		FILE *fp = fopen(txtName, "w");
        // decode
        mavlink_camera_data_t packet;
        mavlink_msg_camera_data_decode(msg, &packet);
		
		snprintf(charBuffer, 256, "time_boot_ms %d\n", packet.time_boot_ms);
		fputs(charBuffer,fp);
		snprintf(charBuffer, 256, "time_week %d\n",packet.time_week);
		fputs(charBuffer,fp);		
		snprintf(charBuffer, 256, "time_week_ms %d\n",packet.time_week_ms);
		fputs(charBuffer,fp);
		snprintf(charBuffer, 256, "roll %f\n",packet.roll);
		fputs(charBuffer,fp);
		snprintf(charBuffer, 256, "pitch %f\n",packet.pitch);
		fputs(charBuffer,fp);
		snprintf(charBuffer, 256, "yaw %f\n",packet.yaw);
		fputs(charBuffer,fp);
		snprintf(charBuffer, 256, "lat %d\n",packet.lat);
		fputs(charBuffer,fp);
		snprintf(charBuffer, 256, "lon %d\n",packet.lon);
		fputs(charBuffer,fp);
		snprintf(charBuffer, 256, "alt %d\n",packet.alt);
		fputs(charBuffer,fp);
		snprintf(charBuffer, 256, "relative_alt %d\n",packet.relative_alt);
		fputs(charBuffer,fp);
		snprintf(charBuffer, 256, "hdg %d\n",packet.hdg);
		fputs(charBuffer,fp);
		snprintf(charBuffer, 256, "fix_type %d\n",packet.fix_type);
		fputs(charBuffer,fp);
		snprintf(charBuffer, 256, "satellites_visible %d\n",packet.satellites_visible);
		fputs(charBuffer,fp);
		
		fclose(fp);
		return true;
		}
	return false;
} // end handle mavlink

void uartSetup()
{
	//-------------------------
	//----- SETUP USART 0 -----
	//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
	
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
}


uint8_t readUart(unsigned char rx_buffer[])
{
	//----- CHECK FOR ANY RX BYTES -----
	if (uart0_filestream != -1)
	{
		// Read up to 255 characters from the port if they are there
		int rx_length = read(uart0_filestream, rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
		if (rx_length < 0)
		{
			//An error occured (will occur if there are no bytes)
			return 0;
		}
		else if (rx_length == 0)
		{
			//No data waiting
			return 0;
		}
		else
		{
			//Bytes received
			rx_buffer[rx_length] = '\0';
			return rx_length;
		}
	}
	return 0;
}


void currnetTime(char buffer[])
{
   time_t rawtime;
   struct tm *info;
   //char buffer[80];

   time( &rawtime );

   info = localtime( &rawtime );


   strftime(buffer,80,"%d%b%g%H%M%S", info);
   printf("Formatted date & time : |%s|\n", buffer );
  
}

void concat(const char *s1, const char *s2, char buffer[])
{
    //in real code you would check for errors in malloc here
    strcpy(buffer, s1);
    strcat(buffer, s2);
}

		
int main(int argc, char **argv)
{
	static uint8_t state;
	static uint8_t cnt;
	unsigned char rx_buffer[256];
	//mavlink
	mavlink_message_t msg;
	mavlink_status_t status;
	const timespec ONE_MILLISECOND = {0, 1000000};


	const char *const args[] = {"raspistill", "-o","temp.jpg", "-t","99999999", "-s", "-q", "20", NULL};
	//"/opt/vc/bin/raspistill"
	int pid = forkAndExecute("raspistill", args);
	//clean up:
	
	setpriority(PRIO_PROCESS, pid, -20);
	setpriority(PRIO_PROCESS, 0, -20);
	printf("PID %d\n", pid);
	
	if (!bcm2835_init())
        printf("GPIO Fail.");
    // Set RPI pin P1-15 to be an input
    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_INPT);
    //  with a pullup
    bcm2835_gpio_set_pud(PIN, BCM2835_GPIO_PUD_DOWN);
	sleep(1);
	
	uartSetup();


	state = WaitForTrigger;
    while (1)
    {
        // Read some data
        uint8_t value = bcm2835_gpio_lev(PIN);
		uint8_t serialNo = readUart(rx_buffer);
		
		//Camera Trigger
		switch(state)
		{
			case WaitForTrigger:
				if (value)
				{
					printf("Picture Triggered\n");
					kill(pid, SIGUSR1);
					cnt = 40;
					state = WaitForData;
				}			
			break;
			case WaitForData:
				//Mavlink
				if(cnt) cnt--;
				
				
				if(cnt == 0 && value)
				{
					printf(ANSI_COLOR_RED "Failed to receive data before trigger\n" ANSI_COLOR_RESET);
					state = PictureFailCleanup;
				}
				else if (serialNo>0)
				{

					for (uint8_t i=0;i<serialNo;i++)
					{
						if (mavlink_parse_char(MAVLINK_COMM_0, rx_buffer[i], &msg, &status))
						{
							// Packet received
							if (handleMessage(&msg))
							{
								printf("Received data\n");
								state = WaitForPicture;
							}
							
						}
					}

				}
			break;
			case WaitForPicture:
				if(cnt) cnt--;
				
				if(cnt == 0 && value)
				{
					printf(ANSI_COLOR_RED "Failed to receive picture before trigger\n" ANSI_COLOR_RESET);
					state = PictureFailCleanup;
				}else				
				if( access( picName, R_OK|W_OK ) != -1 ) 
				{
					int ret;
					char imageTime[80];
					char timeTxtName[100];
					char timePicName[100];
					
					currnetTime(imageTime);
					
					concat(imageTime, ".txt",timeTxtName);
					concat(imageTime, ".jpg",timePicName);
					
					printf("txtName %s\n", timeTxtName);
					printf("jpgName %s\n", timePicName);
						
					
					ret = rename(txtName, timeTxtName);
					if(ret == 0) 
					{
						printf("Text Saved\n");
					}
					else 
					{
						printf("Error: unable to rename Text file\n");
					}
					
					ret = rename(picName, timePicName);

					if(ret == 0) 
					{
						printf("Picture Saved\n");
					}
					else 
					{
						printf("Error: unable to rename Picture file\n");
					}
					
					state = WaitForTrigger;
					sleep(1);
				}
				
			break;
			case PictureFailCleanup:
				sleep(1);
				remove(picName);
				remove(txtName);
				state = WaitForTrigger;
			break;
			default:
			break; 
		}
		nanosleep(&ONE_MILLISECOND, NULL);
	}
	kill(pid,SIGKILL);
    bcm2835_close();
	//----- CLOSE THE UART -----
	close(uart0_filestream);
    return 0;
}
		


int forkAndExecute (const char * path, const char *const args[]) {
     pid_t  pid;
     // stupid execvp
     char * * const non_const_args = (char * * const)args;

     if ((pid = fork()) < 0) {     /* fork a child process           */
          printf("*** ERROR: forking child process failed\n");
          return 0;
     }
     else if (pid == 0) {          /* for the child process:         */
          if (execvp(path, non_const_args) < 0) {     /* execute the command  */
               printf("*** ERROR: exec failed\n");
               return 0;
          }
     }

	return pid;
}

