#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <errno.h>    /* Error number definitions */
#include <sys/ioctl.h>
#include <getopt.h>
#include <pthread.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

#include "gyro_control.h"
#include "nico_motors.h"
#include "main.h"
#include "grip_positions.h"


// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_MOVING_SPEED            32
#define ADDR_MX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting

#define DEMO_MOTORS			22
int DXL_ID[] =                          { 47, 37, 46, 36, 44, 34, 45, 35, 33, 43, 31, 41, 1, 2, 3, 4, 21, 22, 5, 6, 19, 20 };                   // Dynamixel ID: 1
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
                               // Dynamixel will rotate between this value
int DXL_ONE_POSITION_VALUE[] = { 3000, 3000, 3000, 3000, 4000, 4000, 3000, 3000, 4080, 4080, 4080, 4080, 3500, 20, 2850, 2100, 2900, 1500, 950, 2020, 1030, 1600 };                
                               // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
int DXL_TWO_POSITION_VALUE[] = { 0, 0, 8, 8, 0, 0, 0, 0, 20, 20, 10, 10, 600, 2800, 2100, 2850, 1150, 2500, 2020, 950, 3110, 2900 };               

int DXL_INIT_VALUE[] = { 0, 0, 0, 0, 0, 0, 0, 0, 2000, 2000, 1835, 1835, 2048, 1900, 2200, 1900, 2063, 1935, 2020, 3100, 2020, 2080 };               

#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define MIN_REQUIRED_POSE_RECORD_PERIOD   40

static int port_num;
static int dxl_comm_result = COMM_TX_FAIL;             // Communication result
static uint8_t dxl_error = 0;                          // Dynamixel error 

static int current_motor_positions[DEMO_MOTORS];
static int wished_motor_positions[DEMO_MOTORS];


static int16_t maximum_motor_delta = 12;
static int16_t motor_init_accuracy = 80;
static int16_t motor_init_delta = 40;

static FILE *recording_fd;
static FILE *replaying_fd;
static volatile uint8_t is_recording = 0;
static volatile uint8_t is_replaying = 0;
static uint64_t time_recording_started;
static uint64_t time_replay_started;

static volatile uint8_t is_gripping = 0;
static volatile uint8_t update_disabled = 0;
static int current_grip_number = 0;

void init_current_positions()
{
	for (int i = 0; i < DEMO_MOTORS; i++)
	{
	    current_motor_positions[i] = DXL_INIT_VALUE[i];
	    wished_motor_positions[i] = current_motor_positions[i];
	}
}

// position is 0-1 for the available range for that particular motor
void nico_move_to_position(int which_motor, double position)
{
	int calculated_position = (int)(0.5 + DXL_ONE_POSITION_VALUE[which_motor] + 
	                                (DXL_TWO_POSITION_VALUE[which_motor] - DXL_ONE_POSITION_VALUE[which_motor]) * position);
	                                
	wished_motor_positions[which_motor] = calculated_position;
}

void nico_set_motor(int which_motor, int new_value)
{
	int dxl_comm_result ;

	write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[which_motor], ADDR_MX_GOAL_POSITION, new_value);
	if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
	{
	  printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
	}
	else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
	{
	  printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
	}
	usleep(5000);		
}

void nico_set_speed(int which_motor, int speed)
{
	write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[which_motor], ADDR_MX_MOVING_SPEED, speed);
	if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
	{
	  printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
	}
	else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
	{
	  printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
	}
}

void add_pose_to_record()
{
	static uint64_t time_of_last_recorded_pose = 0;
	
	uint64_t current_time = msec();
	
	if (current_time - time_of_last_recorded_pose < MIN_REQUIRED_POSE_RECORD_PERIOD) return;
	time_of_last_recorded_pose = current_time;
	
	fprintf(recording_fd, "%lu", msec() - time_recording_started);
	for (int which_motor = 0; which_motor < DEMO_MOTORS; which_motor++)
	  fprintf(recording_fd, " %d", current_motor_positions[which_motor]);
	fprintf(recording_fd, "\n");	
}

void slowly_approach_current_position()
{
  for (int i = 0; i < DEMO_MOTORS; i++)
  {	  
	  int dxl_present_position;
	  do {
		dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[i], ADDR_MX_PRESENT_POSITION);
		if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
		{
			printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
		}
		else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
		{
			printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
		}	

		int delta = motor_init_delta;
		if (i < 8) delta = 4 * motor_init_delta;
		else if (is_gripping) break;
		
		if (i == 12) delta *= 2;
		
		if (current_motor_positions[i] > dxl_present_position) dxl_present_position += delta;
		else if (current_motor_positions[i] < dxl_present_position) dxl_present_position -= delta;
		
		printf("%d %d -> %d\r", i, dxl_present_position, DXL_INIT_VALUE[i]);
		
	    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[i], ADDR_MX_GOAL_POSITION, dxl_present_position);
		if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
		{
		  printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
		}
		else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
		{
		  printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
		}
		
		usleep(5000);
	} while (abs(dxl_present_position - current_motor_positions[i]) > motor_init_accuracy * 2);	
	printf("reached current position.\n");
  }
}


// position is 0-1 for the available range for that particular motor
void nico_update_motor_positions()
{
//	static uint64_t last_report = 0;
	
	uint8_t at_least_one_motor_has_changed = 0;
	
	for (int which_motor = 0; which_motor < DEMO_MOTORS; which_motor++)
	{	
		int change = 0;
		int delta = maximum_motor_delta;
		if (which_motor < 8) delta = 4 * maximum_motor_delta;
		else if (is_gripping) break;
		
		if (current_motor_positions[which_motor] < wished_motor_positions[which_motor])
		{
			if (current_motor_positions[which_motor] + delta < wished_motor_positions[which_motor])
			  current_motor_positions[which_motor] += delta;
			else current_motor_positions[which_motor] = wished_motor_positions[which_motor];
			change = 1;
		}
		else if (current_motor_positions[which_motor] > wished_motor_positions[which_motor])
		{
			if (current_motor_positions[which_motor] - delta > wished_motor_positions[which_motor])
			  current_motor_positions[which_motor] -= delta;
			else current_motor_positions[which_motor] = wished_motor_positions[which_motor];
			change = 1;
		}
	    
		// Write goal position
		if (change)
		{
			at_least_one_motor_has_changed = 1;
			nico_set_motor(which_motor, current_motor_positions[which_motor]);
		}
	}
	
	if (at_least_one_motor_has_changed && is_recording)
		add_pose_to_record();
}

int16_t nico_get_motor_position(int which_motor)
{
    int16_t dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[which_motor], ADDR_MX_PRESENT_POSITION);
	if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
	{
		printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
	}
	else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
	{
		printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
	}	
	return dxl_present_position;
}

double nico_get_current_position(int which_motor)
{
    double 	dxl_present_position = nico_get_motor_position(which_motor);
    
	if (DXL_ONE_POSITION_VALUE[which_motor] < DXL_TWO_POSITION_VALUE[which_motor])
	{
		if (dxl_present_position < DXL_ONE_POSITION_VALUE[which_motor])
		   dxl_present_position = DXL_ONE_POSITION_VALUE[which_motor];
		if (dxl_present_position > DXL_TWO_POSITION_VALUE[which_motor])
		   dxl_present_position = DXL_TWO_POSITION_VALUE[which_motor];
	}
	else
	{
		if (dxl_present_position > DXL_ONE_POSITION_VALUE[which_motor])
		   dxl_present_position = DXL_ONE_POSITION_VALUE[which_motor];
		if (dxl_present_position < DXL_TWO_POSITION_VALUE[which_motor])
		   dxl_present_position = DXL_TWO_POSITION_VALUE[which_motor];
	}
	
	return (dxl_present_position - DXL_ONE_POSITION_VALUE[which_motor]) / (DXL_TWO_POSITION_VALUE[which_motor] - DXL_ONE_POSITION_VALUE[which_motor]);
}

/*
	do
	{
	  // Read present position
	  dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[which_motor], ADDR_MX_PRESENT_POSITION);
	  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
	  {
		printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
	  }
	  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
	  {
		printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
	  }
    } while ((abs(calculated_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
     */

void do_replay_recording()
{
	int ln = 2;
    uint64_t t;
    char buf[256];
	 
	
	do {
		
		// skip header line
		if (0 == fgets(buf, 255, replaying_fd))
		{
			printf("error reading recording on line %d\n", ln);
			break;
		}
		
		printf("moving to recording starting position...\n");
		
		int nread = fscanf(replaying_fd, "%lu", &t);
		if (nread != 1)
		{
			printf("error reading recording on line %d\n", ln);
			break;
		}
			
	    int m = 0;
	    int motor_value;
		for (m = 0; m < DEMO_MOTORS; m++)
		{
			if (fscanf(replaying_fd, "%d", &motor_value) != 1)
			{
				printf("error reading recording on line %d\n", ln);
				break;
			}
			current_motor_positions[m] = motor_value;
		}
		if (m < DEMO_MOTORS) break;
	
		slowly_approach_current_position();
		
		printf("starting replay...\n");
	    time_replay_started = msec();

		while ((nread = fscanf(replaying_fd, "%lu", &t)) != EOF)
		{
			if (nread != 1)
			{
				printf("error reading recording on line %d\n", ln);
				break;
			}
			while (!terminated && (msec() - time_replay_started < t));
			if (terminated) break;
			
			int m = 0;
			for (m = 0; m < DEMO_MOTORS; m++)
			{
				if (1 != fscanf(replaying_fd, "%d", &motor_value))
				{
				    printf("error reading recording on line %d\n", ln);
				    break;
			    }
			    if (motor_value != current_motor_positions[m]) 
			    {
					current_motor_positions[m] = motor_value;
				    nico_set_motor(m, motor_value);
				}				  
			}		
			if (m < DEMO_MOTORS) break;
			
			ln++;
		}
		
		printf("completed. please move the hand to some friendly position (5 sec)...\n");
		sleep(5);
		
		for (int m = 0; m < DEMO_MOTORS; m++)
		{
			current_motor_positions[m] = wished_motor_positions[m];
		}
		printf("slowly moving to current hand position...\n");
		slowly_approach_current_position();
        printf("done.\n");
	
	} while(0);
	
	is_replaying = 0;
	fclose(replaying_fd);
}

void *nico_motor_thread(void *args)
{
	printf("Nico motor thread started.\n");
	while (!terminated)
	{
		if (is_replaying) do_replay_recording();
		
		if (!update_disabled) nico_update_motor_positions();
		usleep(10000);		
	}
	
	printf("Nico motor thread finished.\n");
	return 0;
}


// returns 0 on error, otherwise 1
int initialize_nico_hand()
{
  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  port_num = portHandler(DEVICENAME);

  // Initialize PacketHandler Structs
  packetHandler();

  dxl_comm_result = COMM_TX_FAIL;             // Communication result

  dxl_error = 0;                          // Dynamixel error

  // Open port
  if (openPort(port_num))
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    //getch();
    return 0;
  }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    //getch();
    return 0;
  }

  // Enable Dynamixel Torque
  for (int i = 0; i < DEMO_MOTORS; i++)
  {
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[i], ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
      printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
      printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
    else
    {
      printf("Dynamixel has been successfully connected \n");
    }
    nico_set_speed(i, 40);
  }  
	
  init_current_positions();
  printf("initializing Nico motor positions...\n");
  // Write init position
  slowly_approach_current_position();
	  
   pthread_t tid;
   pthread_create(&tid, 0, nico_motor_thread, 0);
   pthread_detach(tid);
  
  return 1;
}


void disconnect_nico()
{
	// Disable Dynamixel Torque
  for (int i = 0; i < DEMO_MOTORS; i++)
  {
      //printf("disabling torque of %d\n", DXL_ID[i]);
      write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[i], ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
      if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      }
      else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      }
  }

  // Close port
  closePort(port_num);
  printf("Nico disconnected.\n");
}


void recording_start(char *filename)
{
    if (is_replaying)
	{
		printf("cannot record while replaying\n");
		return;
	}

	recording_fd = fopen(filename, "w+");
	if (recording_fd == 0)
	{
		perror("could not open recording file");
		return;
	}
	is_recording = 1;
	fprintf(recording_fd, "# dynamixel ids: ");
	for (int which_motor = 0; which_motor < DEMO_MOTORS; which_motor++)
	  fprintf(recording_fd, "%d ",  DXL_ID[which_motor]);
	fprintf(recording_fd, "\n");
	time_recording_started = msec();
}

void recording_stop()
{
	is_recording = 0;
	fclose(recording_fd);
}

void replay_recording(char *filename)
{
	if (is_recording)
	{
		printf("cannot replay while recording\n");
		return;
	}
	
	replaying_fd = fopen(filename, "r");
	if (replaying_fd == 0)
	{
		perror("could not open recording file");
		return;
	}
	is_replaying = 1;
}


void nico_grip(int grip_number)
{
	current_grip_number = grip_number;	
	update_disabled = 1;
	usleep(500000);
	
	printf("moving to ready postion...\n");
	for (int m = 0; m < DEMO_MOTORS; m++)
		current_motor_positions[m] = ready_for_grips[grip_number][m];
	slowly_approach_current_position();
	printf("moving to grip postion...\n");
	for (int m = 0; m < DEMO_MOTORS; m++)
		current_motor_positions[m] = grips[grip_number][m];
	slowly_approach_current_position();	
	printf("ready...\n");
	is_gripping = 1;
	update_disabled = 0;
}

void nico_finish_grip()
{
	update_disabled = 1;	
	usleep(500000);
	is_gripping = 0;
	for (int m = 0; m < DEMO_MOTORS; m++)
		current_motor_positions[m] = ready_for_grips[current_grip_number][m];
	slowly_approach_current_position();
	
	printf("move the hand please...\n");
	sleep(5);
	
	printf("moving to your hand position...\n");
	for (int m = 0; m < DEMO_MOTORS; m++)
		current_motor_positions[m] = wished_motor_positions[m];
	slowly_approach_current_position();
	
	printf("ready.\n");
	
	update_disabled = 0;
}

void nico_show()
{
	update_disabled = 1;
	usleep(500000);

    int16_t pos = nico_get_motor_position(r_shoulder_fwd_bwd);
    nico_set_speed(r_shoulder_fwd_bwd, 40);
	nico_set_motor(r_shoulder_fwd_bwd, 3250);
	sleep(6);
	nico_set_motor(r_shoulder_fwd_bwd, pos);
	
	update_disabled = 0;
}
