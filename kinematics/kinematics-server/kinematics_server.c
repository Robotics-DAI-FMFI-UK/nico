#include <fcntl.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h> 
#include <pthread.h>
#include <signal.h>

#include "kinematics_server.h"
#include "nico_motors.h"
#include "main.h"
 
#define LOG_FILENAME "server.log"
 
volatile int connfd = 0;
volatile int thread_started = 0;
int tcounter = 0;
char ip_addr[INET_ADDRSTRLEN + 1];
pthread_mutex_t log_lock;
 
void logmsg(const char *s)
{
    time_t ticks;  
    ticks = time(NULL);
    pthread_mutex_lock(&log_lock);
    FILE *f = fopen(LOG_FILENAME, "a+");
    fprintf(f, "%.24s: %s\n", ctime(&ticks), s);
    fclose(f);
    pthread_mutex_unlock(&log_lock);
}
 
int receive_data(int fd, uint8_t *buf, int len, char *ip)
{
    char logbuf[100];
    int rp = 0;
    while (!terminated && (rp < len))
    {
      int received = recv(fd, buf + rp, len - rp, 0);
      if (received == -1)
      {
        if ((errno == EAGAIN) || (errno == EWOULDBLOCK))
        {
	    usleep(1000);
	    continue;
        }
        sprintf(logbuf, "recv() error %d from %s", errno, ip);
        logmsg(logbuf);
        shutdown(fd, SHUT_RDWR);
	sleep(1);
        close(fd);
        return 0;
      }
      else if (received == 0)
      {
        sprintf(logbuf, "recv() -> 0, other side orderly shut down %s", ip);
        logmsg(logbuf);
        shutdown(fd, SHUT_RDWR);
        sleep(1);
        close(fd);
        return 0;
      }
 
      rp += received;
    }
    return !terminated;
}

void handle_packet(uint8_t *packet, int16_t packet_size)
{
    char logm[100];
    if (packet[0] == PACKET_SET_ANGLES)
    {
        if (packet_size != 2 + 2 * MOTOR_DATA_COUNT)
	{
	    sprintf(logm, "set angles packet of incorrect size %d, expected: %d", packet_size, 2 + 2 * MOTOR_DATA_COUNT);
            logmsg(logm);
	    return;
	}
        motor_data *motors = (motor_data *)(packet + 2);
        nico_set_wished_motor_positions(motors);
    } 
}

void *handle_client(void *args)
{
    int fd = connfd;
    uint8_t *recvbuf;
    char ip[INET_ADDRSTRLEN + 1];
    strncpy(ip, ip_addr, INET_ADDRSTRLEN);
    ip[INET_ADDRSTRLEN] = 0;
    thread_started = 1;
 
    struct timeval tv;
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
 
    while (!terminated)
    {
        uint16_t packet_size;
        if (!receive_data(fd, (uint8_t *)&packet_size, 2, ip)) return 0;

	recvbuf = (uint8_t *) malloc(packet_size);
	if (!receive_data(fd, recvbuf, packet_size, ip)) return 0;

	handle_packet(recvbuf, packet_size);
	free(recvbuf);
    }

    shutdown(fd, SHUT_RDWR);            
    sleep(1);
    close(fd);
    return 0;
}

static int kinematics_server_tcpport;


void *kinematics_server(void *args)
{
    int listenfd = 0;
    struct sockaddr_in serv_addr; 
    char buf[200];

    logmsg("starting server");
    
    pthread_mutex_init(&log_lock, 0);
    
    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    if (listenfd == -1)
    {
        sprintf(buf, "error socket(): %d\n", errno);
        logmsg(buf);
        return 0;
    }
    memset(&serv_addr, '0', sizeof(serv_addr));
    
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(kinematics_server_tcpport);
 
    int yes = 1;
    if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, (void*)&yes, sizeof(yes)) < 0) 
    {
      fprintf(stderr, "setsockopt() failed. Error: %d\n", errno);
    }

    if (-1 == bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)))
    {
        sprintf(buf, "error bind(): %d\n", errno);
        logmsg(buf);
        return 0;
    }
 
    if (-1 == listen(listenfd, 10))
    {
        sprintf(buf, "error listen(): %d\n", errno);
        logmsg(buf);
        return 0;
    }
 
    printf("listening on port %d...\n", kinematics_server_tcpport);
    while(!terminated)
    {
        struct sockaddr client_addr;
        socklen_t clen = sizeof(struct sockaddr);
 
        connfd = accept(listenfd, &client_addr, &clen); 
        if (connfd == -1) 
        {
            logmsg("accept() returned error");
            continue;
        }
    
        struct sockaddr_in* pV4Addr = (struct sockaddr_in*)&client_addr;
        struct in_addr ipAddr = pV4Addr->sin_addr;
        inet_ntop( AF_INET, &ipAddr, ip_addr, INET_ADDRSTRLEN );
        char buf[100];
        sprintf(buf, "accepted %s", ip_addr);
        logmsg(buf);
    
        thread_started = 0;
        tcounter++;
        pthread_t tid;
        if (pthread_create(&tid, 0, handle_client, 0))
        {
            logmsg("pthread_create returned error");
            continue;
        }
    
        pthread_detach(tid);
        while (!thread_started) usleep(10);
    }
    close(listenfd);
    printf("kinematics server terminated.\n");
    return 0;
}

int init_kinematics_server(int tcpport)
{
    kinematics_server_tcpport = tcpport;
    pthread_t tid;

    if (pthread_create(&tid, 0, kinematics_server, 0))
    {
        logmsg("pthread_create returned error");
        return 0;
    }
    pthread_detach(tid);
    return 1;
}

