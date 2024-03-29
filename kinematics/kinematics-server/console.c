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

static char recording_file_name[MAX_REC_FILE_NAME + 1];
static int recording_file_index;
static uint64_t recording_time_started;

void print_help()
{
	printf("help      print this help\n");
	printf("exit      exit application\n");
	printf("file name setup file name for recording\n");
	printf("save      start recording\n");
	printf("play #    replay saved recording number\n");
	printf("done      stop recording\n");
}	

void *console_thread(void *args)
{
    printf("Kinematics server console, type exit to leave...\n");
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
      else if (strncmp(buf, "file", 4) == 0)
      {
		  if (strlen(buf) >= 6)
		  {
			  strncpy(recording_file_name, buf + 5, MAX_REC_FILE_NAME);
			  recording_file_index = 0;
		  }
	  }
	  else if (strncmp(buf, "save", 4) == 0)
	  {
		  char *recording_file = (char *)malloc(strlen(recording_file_name) + 1 + 4 + 1);
		  sprintf(recording_file, "%s_%04d", recording_file_name, recording_file_index);		  
		  recording_start(recording_file);
		  free(recording_file);
		  printf("started recording %s #%d\n", recording_file_name, recording_file_index);		  
		  recording_time_started = msec();
	  }
	  else if (strncmp(buf, "done", 4) == 0)
	  {
		  recording_stop();
		  printf("finished recording %s #%d (%lu ms)\n", recording_file_name, recording_file_index, msec() - recording_time_started);
		  recording_file_index++;
	  }
	  else if (strncmp(buf, "play", 4) == 0)
	  {
		  int recording_number = -1;
		  sscanf(buf + 5, "%d", &recording_number);
		  if (recording_number < 0) continue;
		  char *recording_file = (char *)malloc(strlen(recording_file_name) + 1 + 4 + 1);
		  sprintf(recording_file, "%s_%04d", recording_file_name, recording_number);
		  replay_recording(recording_file);
		  free(recording_file);		  
	  }
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

