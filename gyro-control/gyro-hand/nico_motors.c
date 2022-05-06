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


// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
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


static int port_num;
static int dxl_comm_result = COMM_TX_FAIL;             // Communication result
static uint8_t dxl_error = 0;                          // Dynamixel error 

static int current_motor_positions[DEMO_MOTORS];
static int wished_motor_positions[DEMO_MOTORS];


static int16_t maximum_motor_delta = 12;
static int16_t motor_init_accuracy = 80;
static int16_t motor_init_delta = 40;

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

// position is 0-1 for the available range for that particular motor
void nico_update_motor_positions()
{
//	static uint64_t last_report = 0;
	
	int dxl_comm_result ;
	
	for (int which_motor = 0; which_motor < DEMO_MOTORS; which_motor++)
	{	
		int change = 0;
		int delta = maximum_motor_delta;
		if (which_motor < 8) delta = 4 * maximum_motor_delta;
		
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
			write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[which_motor], ADDR_MX_GOAL_POSITION, current_motor_positions[which_motor]);
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
	}
}

double nico_get_current_position(int which_motor)
{
    double dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[which_motor], ADDR_MX_PRESENT_POSITION);
	if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
	{
		printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
	}
	else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
	{
		printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
	}	
	
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


void *nico_motor_thread(void *args)
{
	printf("Nico motor thread started.\n");
	while (!terminated)
	{
		nico_update_motor_positions();
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
  }  
	
  init_current_positions();
  printf("initializing Nico motor positions...\n");
  // Write init position
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
		
		if (DXL_INIT_VALUE[i] > dxl_present_position) dxl_present_position += motor_init_delta;
		else if (DXL_INIT_VALUE[i] < dxl_present_position) dxl_present_position -= motor_init_delta;
		
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
	} while (abs(dxl_present_position - DXL_INIT_VALUE[i]) > motor_init_accuracy * 2);
	
  }
  
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


