#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "pngwriter.h"

//odkomentujte nasledovny riadok ak kamera nepodporuje BGR format
//pozri v4l2-ctl -d /dev/videoX --list-formats

#define POUZI_YUV

uint8_t *buffer0;
uint8_t *buffer1;
 
static int xioctl(int fd, int request, void *arg)
{
        int r;
 
        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);
 
        return r;
}
 
int setup_format(int fd)
{
        struct v4l2_format fmt = {0};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        //fmt.fmt.pix.width = 640;
        //fmt.fmt.pix.height = 480;
        fmt.fmt.pix.width = 4208;
        fmt.fmt.pix.height = 3120;

        // ak vasa kamera nepodporuje BGR24, mozno podporuje YUV420,
        // ale v tom pripade bude treba obrazok spracovavat v tom
        // formate, alebo si ho skonvertovat...

#ifdef POUZI_YUV
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
#else
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
#endif

        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        
        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
        {
            perror("nepodarilo sa nastavit format");
            return 1;
        }
 
        return 0;
}
 
int init_mmap(int fd, int id)
{
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
 
    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        perror("nepodarilo sa inicializovat mmap buffer");
        return 1;
    }
 
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
    {
        perror("nepodarilo sa vytiahnut smernik na mmap buffer");
        return 1;
    }
 
    uint8_t **buffer = (id)?(&buffer1):(&buffer0);
    *buffer = mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
 
    return 0;
}
 
int capture_images(int fd0, int fd1)
{
    struct v4l2_buffer buf0 = {0};
    struct v4l2_buffer buf1 = {0};
    buf0.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf1.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(-1 == xioctl(fd0, VIDIOC_STREAMON, &buf0.type))
    {
        perror("nepodarilo sa zapnut snimanie kamery");
        return 1;
    }
    if(-1 == xioctl(fd1, VIDIOC_STREAMON, &buf1.type))
    {
        perror("nepodarilo sa zapnut snimanie kamery");
        return 1;
    }

    // do suborov ukladame obrazky
    do
    {
      char s[10];
      printf("Stlac ENTER alebo napis 'exit'...\n");
      fgets(s, 10, stdin);
      if (strncmp(s, "exit", 4) == 0) break;

      
      sleep(1);
      for (int sec = 3; sec >0; sec--)
      {
	printf("%d...\n", sec);
        sleep(1);
      }
      
      printf("vtacik\n");
       

      buf0.memory = V4L2_MEMORY_MMAP;
      buf0.index = 0;
      buf1.memory = V4L2_MEMORY_MMAP;
      buf1.index = 0;

      if(-1 == xioctl(fd0, VIDIOC_QBUF, &buf0))
      {
          perror("nepodarilo sa poziadat o mmap buffer");
          return 1;
      }
      if(-1 == xioctl(fd1, VIDIOC_QBUF, &buf1))
      {
          perror("nepodarilo sa poziadat o mmap buffer");
          return 1;
      }
   
      fd_set fds;
      do {
        FD_ZERO(&fds);
        FD_SET(fd0, &fds);
        FD_SET(fd1, &fds);
        struct timeval tv = {0};
        tv.tv_sec = 2;

        int r = select(fd1+1, &fds, NULL, NULL, &tv);

        if(-1 == r)
        {
          perror("pocas cakania na dalsi obrazok doslo k chybe");
          return 1;
        }
      } while (!FD_ISSET(fd0, &fds) || !FD_ISSET(fd1, &fds));
   
      if(-1 == xioctl(fd0, VIDIOC_DQBUF, &buf0))
      {
          perror("nepodarilo sa ziskat dalsi obrazok");
          return 1;
      }
      if(-1 == xioctl(fd1, VIDIOC_DQBUF, &buf1))
      {
          perror("nepodarilo sa ziskat dalsi obrazok");
          return 1;
      }
      printf ("mam v pamati, zapisujem par obrazkov...\n");
  
      static int counter = 0;
      char filename[30];
      sprintf(filename, "image_l_%d.png", counter);
      write_uyvy_png_image((uint8_t *)buffer0, filename, 4208, 3120);
      
      sprintf(filename, "image_r_%d.png", counter++);
      write_uyvy_png_image((uint8_t *)buffer1, filename, 4208, 3120);
    } while (1);
    printf("exiting...\n");

    return 0;
}
 
int main(int argc, char **argv)
{
        int fd0, fd1;
        // ak mate pripojenych viac kamier, moze to byt napr. /dev/video2 

	const char *device0 = "/dev/video0";
	const char *device1 = "/dev/video2";
 
        fd0 = open(device0, O_RDWR);
        fd1 = open(device1, O_RDWR);
        if ((fd0 == -1) || (fd1 == -1))
        {
                perror("nepodarilo sa otvorit zariadenie /dev/videoN...");
                return 1;
        }

        if(setup_format(fd0))
            return 1;
        if(setup_format(fd1))
            return 1;
        
        if(init_mmap(fd0, 0))
            return 1;
        if(init_mmap(fd1, 1))
            return 1;

        if(capture_images(fd0, fd1))
            return 1;

        close(fd0);
        close(fd1);
        return 0;
}
