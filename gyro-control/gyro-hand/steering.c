#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "gyro_control.h"
#include "nico_motors.h"

#define USE_RIGHT_HAND

#ifdef USE_LEFT_HAND
int limb_index_to_motor_index[] = {	l_shoulder_left_right, 
	                                l_shoulder_lift, 
	                                l_shoulder_fwd_bwd, 
	                                l_elbow, 
	                                l_wrist_rotate, 
	                                l_wrist_left_right, 
	                                l_index_finger, 
	                                l_other_fingers, 
	                                l_thumb_lift, 
	                                l_thumb_close };
#endif

#ifdef USE_RIGHT_HAND
int limb_index_to_motor_index[] = {	r_shoulder_fwd_bwd,	                                 
									r_shoulder_left_right,
									r_shoulder_lift, 
	                                r_elbow, 
	                                r_wrist_rotate, 
	                                r_wrist_left_right, 
	                                r_index_finger, 
	                                r_other_fingers, 
	                                r_thumb_lift, 
	                                r_thumb_close };
#endif	                                
	                  

int limb_max[] = { 90,60,30,50,80,28,300,230,300,60 }; //bent
int limb_min[] = { -90,-60,0,0,-80,-28,0,0,0,0 };
//filePointer = fopen("log.txt", "a+") ;	
void move_all_limbs(hand_data *old_hand, hand_data *new_hand)
{	
	for (int i = 0; i < HAND_DATA_COUNT; i++)
	{		
		//if (i < 4) continue;	
	
	    //if (i != 5) continue;	
	    //printf("%d\n", new_hand->data[i]);
		int16_t limb_diff = abs(old_hand->data[i] - new_hand->data[i]);
		if (limb_diff >= 60)
		{
			new_hand->data[i] = old_hand->data[i] + (new_hand->data[i] - old_hand->data[i]) / 3;
		}		
		
		if (limb_diff > 3) //&& (limb_diff < 60))
	    {
		    old_hand->data[i] = new_hand->data[i];
		    
		    int mag = (new_hand->data[i] - limb_min[i]);
		    if (mag < 0) mag = 0;			
			if (mag > limb_max[i] - limb_min[i]) mag = limb_max[i] - limb_min[i];
					    
		    double motor_value = 1.0 - mag / (double)(limb_max[i] - limb_min[i]);
		    		    
			//printf("%d: %d -> %d -> %.2lf\n", i, new_hand->data[i], mag, motor_value);
			
		    nico_move_to_position(limb_index_to_motor_index[i], motor_value);
		}
	}
}

void *steering(void *args)
{
	 hand_data previous_hand;
	 get_gyro_orientation(&previous_hand);
	 
     while (!terminated) 
     {
	     hand_data hand;
	     get_gyro_orientation(&hand);
	     
	     move_all_limbs(&previous_hand, &hand); 
	     
		 usleep(50000);
	     //sleep(1);
     }
     return 0;
}

int init_steering()
{
    pthread_t tid;
    if (pthread_create(&tid, 0, steering, 0)) 
    {   
        perror("pthread_create returned error");
	return 0;
    }   
    pthread_detach(tid);
    return 1;
}


/*
	int oa[6];
	int a[6];
	char buf[40];
	int parts[] = {	l_other_fingers, l_index_finger, l_thumb_lift, l_thumb_close, l_wrist_left_right, l_wrist_rotate };
	for (int i = 0; i < 6; i++)	oa[i] = -1;
	int max[] = { 580,550,780,790,0,0 };//bended
	int min[] = { 240,250,540,505,0,0 };
	
	
     while (!terminated) 
     {
            serialport_read_until(arduino_fd, buf, '\n');
            //printf("read %d bytes: %s", n, buf);            
			sscanf(buf, "%d%d%d%d%d%d", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]);
			
			for (int i = 0; i < 6; i++)
			{
				if ((oa[i] == -1) || ((abs(a[i] - oa[i]) > 20) && (abs(a[i] - oa[i]) < 200)))
				{
				  int mag = (a[i] - min[i]);
				  if (mag < 0) mag = 0;
				  if (mag > max[i] - min[i]) mag = max[i] - min[i];
				  
				  printf("%d: %d -> %d -> %.2lf\n", i, a[i], mag, 1.0 - mag / (double)(max[i]-min[i]));
				  
				  nico_move_to_position(parts[i], 1.0 - mag / (double)(max[i]-min[i]));   
				  oa[i] = a[i];
				}
			}
			if (testing)
			{
				printf("%s", buf);
			}
			
     }
}

*/
