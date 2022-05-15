#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <errno.h>    /* Error number definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>

#include "gyro_control.h"
#include "main.h"

static int serialport_init(const char* serialport, int baud);
static int serialport_write(int fd, const char* str);
static int serialport_read_until(int fd, char* buf, char until, int max_to_read);
static int serialport_skip_until(int fd, char until);

static int arduino_fd;

// floating average rate
static double alpha = 0.25;

static hand_data last_data;
static pthread_mutex_t arrive_data_lock;

int initialize_arduino(char *serialport)
{
    int fd = 0;
    int baudrate = B115200;  // default    

      baudrate = 9600;
      fd = serialport_init(serialport, baudrate);
      if(fd==-1) return -1;
      usleep(3000 * 1000);    
      serialport_write(fd, "1");
      serialport_skip_until(fd, '@');
      printf("arduino handshake...\n");
      serialport_skip_until(fd, '*');
      printf("arduino initialized...\n");
      return fd;
}

void *main_loop(void *arg)
{
    int16_t a[6];
    int16_t b[4];
    char buf[40];
    int16_t chksm;
    
     while (!terminated) 
     {
         serialport_read_until(arduino_fd, buf, '\n', 40);
         //printf("read %d bytes: %s", n, buf);            
         sscanf(buf, "%hx%hx%hx%hx%hx%hx%hx%hx%hx%hx%hx", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &b[0], &b[1], &b[2], &b[3], &chksm);
	     if (((a[0] + a[1] + a[2] + a[3] + a[4] + a[5] + b[0] + b[1] + b[2] + b[3]) % 256) != chksm)
	     {
		     static int mismatch_count = 0;
		     printf("\t\t\t\t\t\t\t\tchksm mismatch %d\r", mismatch_count++);
                 //printf(": %hd %hd %hd %hd %hd %hd %hd %hd %hd %hd:  %hd vs %hd\n", a[0], a[1], a[2], a[3], a[4], a[5], b[0], b[1], b[2], b[3], chksm,
	         //			 ((a[0] + a[1] + a[2] + a[3] + a[4] + a[5] + b[0] + b[1] + b[2] + b[3]) % 256));

		     continue;
	      }

		 pthread_mutex_lock(&arrive_data_lock);
		 last_data.data[hand_yaw] = (int16_t)(0.5 + last_data.data[hand_yaw] * (1.0 - alpha) + alpha * (a[0]));
		 last_data.data[hand_roll] = (int16_t)(0.5 + last_data.data[hand_roll] * (1.0 - alpha) + alpha * (a[1]));
		 last_data.data[hand_pitch] = (int16_t)(0.5 + last_data.data[hand_pitch] * (1.0 - alpha) + alpha * (a[2]));
		 last_data.data[hand_elbow] = (int16_t)(0.5 + last_data.data[hand_elbow] * (1.0 - alpha) + alpha * (a[3]));
		 last_data.data[hand_wrist_rot] = (int16_t)(0.5 + last_data.data[hand_wrist_rot] * (1.0 - alpha) + alpha * (-a[5]));
		 		 
		 //last_data.data[hand_yaw] = (int16_t)(0.5 + last_data.data[hand_yaw] * (1.0 - alpha) + alpha * (-a[0]));
		 //last_data.data[hand_roll] = (int16_t)(0.5 + last_data.data[hand_roll] * (1.0 - alpha) + alpha * (-a[1]));
		 //last_data.data[hand_pitch] = (int16_t)(0.5 + last_data.data[hand_pitch] * (1.0 - alpha) + alpha * (a[2]));
		 //last_data.data[hand_elbow] = (int16_t)(0.5 + last_data.data[hand_elbow] * (1.0 - alpha) + alpha * (-a[3]));
		 //last_data.data[hand_wrist_rot] = (int16_t)(0.5 + last_data.data[hand_wrist_rot] * (1.0 - alpha) + alpha * a[5]);
		 
		 last_data.data[hand_wrist_bend] = (int16_t)(0.5 + last_data.data[hand_wrist_bend] * (1.0 - alpha) + alpha * (-a[4]));
		 last_data.data[hand_index_fi] = b[0];
		 last_data.data[hand_middle_fi] = b[1];
		 last_data.data[hand_thumb_raise] = b[2];
		 last_data.data[hand_thumb_bend] = b[3];	 
		 pthread_mutex_unlock(&arrive_data_lock);
     
		 if (testing)
			 printf("%d %d %d %d\n", last_data.data[hand_roll], last_data.data[hand_pitch], last_data.data[hand_yaw], last_data.data[hand_elbow]);
     }
     close(arduino_fd);
     printf("arduino connection closed.\n");
     return 0;
}

void get_gyro_orientation(hand_data *new_data)
{
    pthread_mutex_lock(&arrive_data_lock);
      memcpy(new_data, &last_data, sizeof(hand_data));
    pthread_mutex_unlock(&arrive_data_lock);
}

int init_gyro_input(char *serialport)
{
   pthread_mutex_init(&arrive_data_lock, 0);
    
   printf("connecting to Arduino at %s...\n", serialport);
   arduino_fd = initialize_arduino(serialport);
   if (arduino_fd == -1)
     return 0;    
     
   pthread_t tid;
   pthread_create(&tid, 0, main_loop, 0);
   pthread_detach(tid);
     
  return 1;
}

//arduino communication functions
int serialport_write(int fd, const char* str)
{
    int len = strlen(str);
    int n = write(fd, str, len);
    if (n != len) 
        return -1;
    return n;
}

int serialport_skip_until(int fd, char until)
{
    char b[1];
    int i = 0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
        if (n == -1) return -1;    // couldn't read
        if (n == 0) 
        {
            usleep(10 * 1000); // wait 10 msec try again
            continue;
        }
        i++;
    } while (b[0] != until);
    return i;
}

int serialport_read_until(int fd, char* buf, char until, int max_to_read)
{
    char b[1];
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
        if (n == -1) return -1;    // couldn't read
        if (n == 0) 
        {
            usleep (10 * 1000); // wait 10 msec try again
            continue;
        }
        buf[i] = b[0]; i++;
        if (i == max_to_read) i = 0;
    } while (b[0] != until);

    buf[i] = 0;  // null terminate the string
    return i;
}

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
  
    fd = open(serialport, O_RDWR | O_NOCTTY);
    if (fd == -1)  
    {
        perror("init_serialport: Unable to open port ");
        return -1;
    }
 
    if (tcgetattr(fd, &toptions) < 0) 
    {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }

    speed_t brate = baud; // let you override switch below if needed
    switch(baud) 
    {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
    case 19200:  brate=B19200;  break;
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;
  
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) 
    {
        perror("init_serialport: Couldn't set term attributes");
        return -1;

    }
    return fd;
}

