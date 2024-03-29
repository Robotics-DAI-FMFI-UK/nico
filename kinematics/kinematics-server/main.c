#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>

#include "console.h"
#include "kinematics_server.h"
#include "nico_motors.h"

volatile int terminated = 0;

void sigterm_handler(int i)
{
        return;
}

void setup_sigterm_handler()
{
    struct sigaction act;
    act.sa_handler = sigterm_handler;
    sigemptyset (&act.sa_mask);
    act.sa_flags = 0;
    if (sigaction(SIGTERM, &act, 0))
    {
        perror("sigaction");
        exit(1);
    }
}

uint64_t msec()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return 1000UL * tv.tv_sec + tv.tv_usec / 1000UL;
}

uint64_t usec()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return 1000000UL * tv.tv_sec + tv.tv_usec;
}

void usage()
{
    printf("usage: kinematics_server [options]\n\navailable options:\n  --help print this help\n");
    printf("  --tcpport TCP_PORT    (default is %d)\n\n", KINEMATICS_SERVER_SOCKET_PORT);
    exit(0);
}
 
int main(int argc, char *argv[])
{
    int tcpport = KINEMATICS_SERVER_SOCKET_PORT;

    if ((argc > 1) && (strcmp("--help", argv[1]) == 0)) usage();
    if ((argc > 2) && (strcmp("--tcpport", argv[1]) == 0)) sscanf(argv[2], "%d", &tcpport);

    setup_sigterm_handler();

    init_console();

    if (!initialize_nico_hand())
    {
      printf("could not initialize nico, terminating\n");
      return 1;
    }

    init_kinematics_server(tcpport);

    while  (!terminated)
    { 
        usleep(300000);
    }

    disconnect_nico();
    
    return 0;
}
