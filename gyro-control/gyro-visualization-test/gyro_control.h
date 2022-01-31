#ifndef __GYRO_CONTROL_H__
#define __GYRO_CONTROL_H__

int init_gyro_input();
void get_gyro_orientation(volatile double *x, volatile double *y, volatile double *z);

extern int terminated;

#endif
