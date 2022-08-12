#ifndef _NICO_MOTORS_H_
#define _NICO_MOTORS_H_

#define l_other_fingers 0
#define r_other_fingers 1
#define l_index_finger  2
#define r_index_finger  3
#define l_thumb_lift    4
#define r_thumb_lift    5
#define l_thumb_close   6
#define r_thumb_close   7
#define r_wrist_left_right 8
#define l_wrist_left_right 9
#define r_wrist_rotate  10
#define l_wrist_rotate  11
#define r_shoulder_fwd_bwd 12
#define l_shoulder_fwd_bwd 13
#define r_shoulder_lift 14
#define l_shoulder_lift 15
#define r_shoulder_left_right 16
#define l_shoulder_left_right 17
#define r_elbow 18
#define l_elbow 19
#define head_rotate 20
#define head_lift 21

#include <inttypes.h>

#define MOTOR_DATA_COUNT 22

typedef struct motor_data_str
{
        int16_t data[MOTOR_DATA_COUNT];
} motor_data;


void recording_start(char *fileName);

void recording_stop();

void replay_recording(char *filename); 

// position is 0-1 for the available range for that particular motor
void nico_move_to_position(int which_motor, double position);

// motor data are in the range 0-4095
void nico_set_wished_motor_positions(motor_data *new_motor_data);

// returns 0 on error, otherwise 1
int initialize_nico_hand();

void disconnect_nico();

#endif
