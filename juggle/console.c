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

#include "main.h"
#include "nico_motors.h"

#define MAX_REC_FILE_NAME 200

void print_help()
{
	printf("help      print this help\n");
	printf("exit      exit application\n");
	printf("show      juggle\n");
	printf("loop      juggle in loop\n");
}	

void *console_thread(void *args)
{
    printf("Gyro control console, use gyro to move the plane on the webpage, type exit to leave...\n");
    char buf[25];
    while (!terminated)
    {
      if (fgets(buf, 22, stdin) == 0) break;
      printf("you said: %s", buf);
      char *s = buf;
      while (*s) if ((*s == '\n') || (*s == '\r')) *s = 0; else s++;      
      if (strncmp(buf, "help", 4) == 0)
         print_help();
      else if (strncmp(buf, "exit", 4) == 0)
        terminated = 1;
      else if (strncmp(buf, "show", 4) == 0) nico_show();
      else if (strncmp(buf, "loop", 4) == 0) for (int i = 0; i < 5; i++) nico_show();
    }
    terminated = 1;
    sleep(2);
    kill(getpid(), SIGTERM);  // this signal should be caught so that accept() would be interrupted
    return 0;
}    



int init_console()
{
   pthread_t tid;
   pthread_create(&tid, 0, console_thread, 0);
   pthread_detach(tid);

   return 1;
}

