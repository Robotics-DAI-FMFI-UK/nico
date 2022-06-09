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

#include "nico_motors.h"
#include "main.h"


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

int DXL_INIT_VALUE[] = { 
	538,
517,
9,
362,
19,
15,
0,
965,
2082,
1802,
1828,
1766,
2360,
1782,
2179,
1806,
2175,
1837,
1275,
2825,
1975,
1572
 };

int left_positions1[] = {
538,
517,
9,
362,
19,
15,
0,
965,
2082,
1802,
1828,
1766,
2360,
1782,
2179,
1806,
2175,
1837,
1275,
2825,
1975,
1572
};

int left_positions2[] = {
337,
0,
0,
362,
0,
0,
0,
965,
768,
2679,
2401,
1792,
2029,
1651,
2352,
1950,
2526,
1882,
973,
2601,
1975,
1572
};


int right_positions1[] = {
	538,
517,
9,
362,
19,
15,
0,
965,
2082,
1802,
1828,
1766,
2360,
1782,
2179,
1806,
2175,
1837,
1275,
2825,
1975,
1572
};

int right_positions2[] = {
528,
6,
9,
362,
0,
45,
0,
965,
1415,
2665,
2436,
1581,
2218,
2077,
2309,
1791,
2446,
1740,
1174,
3105,
1975,
1573
};


#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define MIN_REQUIRED_POSE_RECORD_PERIOD   40

static int port_num;
static int dxl_comm_result = COMM_TX_FAIL;             // Communication result
static uint8_t dxl_error = 0;                          // Dynamixel error 

static int current_motor_positions[DEMO_MOTORS];
static int wished_motor_positions[DEMO_MOTORS];


static int16_t maximum_motor_delta = 12;
static int16_t motor_init_accuracy = 20;
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
		
		if (i == 12) delta *= 2;
		
		if (current_motor_positions[i] > dxl_present_position) 
		{
			dxl_present_position += delta;
			if (dxl_present_position > current_motor_positions[i]) dxl_present_position = current_motor_positions[i];
		}
		else if (current_motor_positions[i] < dxl_present_position) 
		{
			dxl_present_position -= delta;
			if (dxl_present_position < current_motor_positions[i]) dxl_present_position = current_motor_positions[i];
		}
		
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
    nico_set_speed(i, 100);
  }  
	
  init_current_positions();
  printf("initializing Nico motor positions...\n");
  // Write init position
  slowly_approach_current_position();
  
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

void nico_show()
{

  for (int k = 0; k < 4; k++)
  {

	  if (k == 0)
             for (int i = 0; i < DEMO_MOTORS; i++)
	        current_motor_positions[i] = left_positions1[i];
	  else if (k == 1)
	  {
             usleep(1000000);
             for (int i = 0; i < DEMO_MOTORS; i++)
	        current_motor_positions[i] = left_positions2[i];
	  }
          else if (k == 2)
             for (int i = 0; i < DEMO_MOTORS; i++)
	        current_motor_positions[i] = right_positions1[i];
          else if (k == 3)
	  {
             usleep(1000000);
             for (int i = 0; i < DEMO_MOTORS; i++)
	        current_motor_positions[i] = right_positions2[i];
	  }


  for (int i = 0; i < DEMO_MOTORS; i++)
  {	  
	  int dxl_present_position;
		
	    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[i], ADDR_MX_GOAL_POSITION, current_motor_positions[i]);
		if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
		{
		  printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
		}
		else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
		{
		  printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
		}
		
		usleep(5000);
	printf("reached current position.\n");
  }
  }
}
