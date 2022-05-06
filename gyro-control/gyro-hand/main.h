#ifndef _MAIN_H_
#define _MAIN_H_

#include <inttypes.h>

extern volatile int terminated;
extern volatile int testing;

uint64_t msec();
uint64_t usec();

#endif
