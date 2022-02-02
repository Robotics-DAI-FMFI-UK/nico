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

static int serialport_init(const char* serialport, int baud);
static int serialport_write(int fd, const char* str);
static int serialport_read_until(int fd, char* buf, char until);
static int serialport_skip_until(int fd, char until);

static int arduino_fd;
static volatile double last_yaw, last_pitch, last_roll;
static pthread_mutex_t ypr_lock;

int initialize_arduino(char *serialport)
{
    int fd = 0;
    int baudrate = B115200;  // default    

      //baudrate = 9600;
      fd = serialport_init(serialport, baudrate);
      if(fd==-1) return -1;
      usleep(3000 * 1000);    
      serialport_write(fd, "1");
      serialport_skip_until(fd, '@');
      printf("arduino initialized...\n");
      return fd;
}

volatile int terminated = 0;
volatile int testing = 0;

void *console_thread(void *args)
{
    printf("Gyro control console, use gyro to move the plane on the webpage, type exit to leave...\n");
    char buf[25];
    while (!terminated)
    {
      if (fgets(buf, 22, stdin) == 0) break;
      printf("you said: %s", buf);
      if (strncmp(buf, "exit", 4) == 0)
        terminated = 1;
      if (strncmp(buf, "test", 4) == 0)
        testing = 1;
      if (strncmp(buf, "stop", 4) == 0)
        testing = 0;
    }
    terminated = 1;
    kill(getpid(), SIGTERM);  // this signal should be caught so that accept() would be interrupted

    return 0;
}    

void *main_loop(void *arg)
{
    double a[3];
    char buf[40];
    
     while (!terminated) 
     {
         serialport_read_until(arduino_fd, buf, '\n');
         //printf("read %d bytes: %s", n, buf);            
         sscanf(buf, "%lf%lf%lf", &a[0], &a[1], &a[2]);
         pthread_mutex_lock(&ypr_lock);
         last_yaw = -a[0];
         last_roll = -a[1];
         last_pitch = -a[2];
         pthread_mutex_unlock(&ypr_lock);
         if (testing)
             printf("%.2f %.2f %.2f\n", last_roll, last_pitch, last_yaw);
     }
     close(arduino_fd);
     return 0;
}

void get_gyro_orientation(volatile double *roll, volatile double *pitch, volatile double *yaw)
{
    pthread_mutex_lock(&ypr_lock);
      *yaw = last_yaw;
      *pitch = last_pitch;
      *roll = last_roll;
    pthread_mutex_unlock(&ypr_lock);
}

int init_gyro_input()
{
   pthread_mutex_init(&ypr_lock, 0);
   char *serialport = "/dev/ttyUSB1";
    
   printf("connecting to Arduino...\n");
   arduino_fd = initialize_arduino(serialport);
   if (arduino_fd == -1)
     return 0;    
     
   pthread_t tid;
   pthread_create(&tid, 0, console_thread, 0);
   pthread_detach(tid);

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

int serialport_read_until(int fd, char* buf, char until)
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

