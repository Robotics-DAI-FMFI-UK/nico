#ifndef __GYRO_CONTROL_H__
#define __GYRO_CONTROL_H__

int init_gyro_input();
void get_gyro_orientation(volatile double *roll, volatile double *pitch, volatile double *yaw);

extern volatile int terminated;

#endif
