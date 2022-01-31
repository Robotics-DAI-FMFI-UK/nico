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

#include "gyro_control.h"
 
#define PLANE_SERVER_SOCKET_PORT 8002
#define LOG_FILENAME "server.log"
 
volatile int connfd = 0;
volatile int thread_started = 0;
pthread_t tid;
int tcounter = 0;
char ip_addr[INET_ADDRSTRLEN + 1];
pthread_mutex_t log_lock;

static volatile double x = 0, y = 0, z = 0;
 
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
 
void *handle_client(void *args)
{
    time_t ticks;
    int fd = connfd;
    char recvbuf[2000];
    int rp = 0;
    int myid = tcounter;
    char ip[INET_ADDRSTRLEN + 1];
    strncpy(ip, ip_addr, INET_ADDRSTRLEN);
    ip[INET_ADDRSTRLEN] = 0;
    thread_started = 1;
 
    struct timeval tv;
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
 
    int newlineseen = 0;
    while (!newlineseen)
    {
      int received = recv(fd, recvbuf + rp, 1000, 0);
      if (received == -1)
      {
        if ((errno == EAGAIN) || (errno == EWOULDBLOCK))
        {
            ticks = time(NULL);
            snprintf(recvbuf, sizeof(recvbuf), "I won't wait for your request any longer.\r\nServer time: %.24s\r\n", ctime(&ticks));
            send(fd, recvbuf, strlen(recvbuf), 0); 
            sleep(1);
        }
        sprintf(recvbuf, "T%d recv() error %d from %s", myid, errno, ip);
        logmsg(recvbuf);
        shutdown(fd, SHUT_RDWR);
	sleep(1);
        close(fd);
        return 0;
      }
      else if (received == 0)
      {
        sprintf(recvbuf, "T%d recv() -> 0, other side orderly shut down %s", myid, ip);
        logmsg(recvbuf);
        shutdown(fd, SHUT_RDWR);
        sleep(1);
        close(fd);
        return 0;
      }
 
      rp += received;
      recvbuf[rp] = 0;
      newlineseen = (strchr(recvbuf, '\n') != 0);
    }
 
    if (strncmp(recvbuf, "GET ", 4) == 0)
    {
	char *space = strchr(&recvbuf[5],' ');
	if(space == 0) {
          shutdown(fd, SHUT_RDWR);
          sleep(1);
          close(fd);
          logmsg(recvbuf);
          logmsg("space not found");
          return 0;
        }
	*space = '\0';
	int filede = -1;
	if (recvbuf[5] == 'x')
	{
          get_gyro_orientation(&x, &y, &z);
	  char orientation[40];
	  sprintf(orientation, "%ddeg %ddeg %ddeg", (int)(x + 0.5), (int)(y + 0.5), (int)(z + 0.5));
          sprintf(recvbuf, "HTTP/1.1 200 OK\r\nServer: gyro-server\r\nContent-Type: text/plain\r\nContent-Length: %ld\r\n\r\n%s", strlen(orientation), orientation);
	}
	else
	{
	  struct stat info;
	  stat(&recvbuf[5],&info);
	  filede = open(&recvbuf[5],O_RDONLY);
	  char *content_type = "text/html";
	  if (recvbuf[5] == 'p') content_type = "model/gltf-binary";
          sprintf(recvbuf, "HTTP/1.1 200 OK\r\nServer: gyro-server\r\nContent-Type: %s\r\nContent-Length: %ld\r\n\r\n", content_type, info.st_size);
	}
        send(fd, recvbuf, strlen(recvbuf), 0);
        if (filede > 0) 
	{
	  	uint8_t buffer[50];
	  	int readin;
	  	while((readin = read(filede, buffer, 50)) > 0){
	  		send(fd, buffer, readin, 0); 
	  	} 
	  	close(filede);
        }
        shutdown(fd, SHUT_RDWR);
	sleep(1);
        close(fd);
        return 0;
    }
 
    recvbuf[14] = 0;
    logmsg(recvbuf);
    sprintf(recvbuf, "T%d: wrong get from %s, closing.", myid, ip);
    logmsg(recvbuf);
    ticks = time(NULL);
    snprintf(recvbuf, sizeof(recvbuf), "Request not recognised\r\n%.24s\r\n", ctime(&ticks));
    send(fd, recvbuf, strlen(recvbuf), 0); 
    shutdown(fd, SHUT_RDWR);            
	sleep(1);
    close(fd);
    return 0;
}
 
int main(int argc, char *argv[])
{
    int listenfd = 0;
    struct sockaddr_in serv_addr; 
    char buf[200];
 
    if (!init_gyro_input())
    {
      printf("could not initialize gyro input, terminating\n");
      return 1;
    }

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
    serv_addr.sin_port = htons(PLANE_SERVER_SOCKET_PORT); 
 
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
 
    printf("listening on port %d...\n", PLANE_SERVER_SOCKET_PORT);
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
        if (pthread_create(&tid, 0, handle_client, 0))
        {
            logmsg("pthread_create returned error");
            continue;
        }
    
        pthread_detach(tid);
        while (!thread_started) usleep(10);
    }
    close(listenfd);
    return 0;
}
