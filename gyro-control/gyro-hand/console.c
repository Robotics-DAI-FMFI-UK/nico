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

