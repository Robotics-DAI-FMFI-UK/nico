#ifndef __GYRO_CONTROL_H__
#define __GYRO_CONTROL_H__

#include <inttypes.h>

#define HAND_DATA_COUNT 10

#define hand_yaw           0
#define hand_pitch         1
#define hand_roll          2
#define hand_elbow         3
#define hand_wrist_bend    5
#define hand_wrist_rot     4
#define hand_index_fi      6
#define hand_middle_fi     7
#define hand_thumb_bend    8
#define hand_thumb_raise   9

    
typedef struct hand_data_str 
{
	int16_t data[HAND_DATA_COUNT];	
} hand_data;

int init_gyro_input(char *serialport);
void get_gyro_orientation(hand_data *new_data);

extern volatile int terminated;

#endif
