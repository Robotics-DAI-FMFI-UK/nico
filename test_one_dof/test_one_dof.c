/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Read and Write Example      *********
//
//
// Available DXL model on this example : All models using Protocol 1.0
// This example is designed for using a Dynamixel MX-28, and an USB2DYNAMIXEL.
// To use another Dynamixel model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below "#define"d variables yourself.
// Be sure that Dynamixel MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
//

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
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library


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

const char *limb_names[] = {
    "l_other_fingers",
    "r_other_fingers",
    "l_index_finger",
    "r_index_finger",
    "l_thumb_lift",
    "r_thumb_lift",
    "l_thumb_close",
    "r_thumb_close",
    "r_wrist_left_right",
    "l_wrist_left_right",
    "r_wrist_rotate",
    "l_wrist_rotate",
    "r_shoulder_fwd_bwd",
    "l_shoulder_fwd_bwd",
    "r_shoulder_lift",
    "l_shoulder_lift",
    "r_shoulder_left_right",
    "l_shoulder_left_right",
    "r_elbow",
    "l_elbow",
    "head_rotate",
    "head_lift" 
};


// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_MOVING_SPEED            32
#define ADDR_MX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting

#define DEMO_MOTORS			22
int DXL_ID[] =                          { 47, 37, 46, 36, 44, 34, 45, 35, 33, 43, 31, 41, 1, 2, 3, 4, 21, 22, 5, 6, 19, 20 }; 
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
int DXL_INIT_POSITION_VALUE[] = { 2500, 2500, 2500, 2500, 3500, 3500, 1500, 1500, 3700, 3700, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800 };   
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main()
{
  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(DEVICENAME);

  // Initialize PacketHandler Structs
  packetHandler();

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_present_position = 0;              // Present position

  // Open port
  if (openPort(port_num))
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
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
    getch();
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

  int positions[DEMO_MOTORS];
  int limb_index = 0;
  do {
  // Read present position
  for (int i = 0; i < DEMO_MOTORS; i++)
  {
      dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[i], ADDR_MX_PRESENT_POSITION);
      if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
         printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      }
      else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
         printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      }
      printf("limb_index:%03d, limb_name: %s, dynamixel_id:%03d, position:%03d\n", i, limb_names[i], DXL_ID[i], dxl_present_position);
      positions[i] = dxl_present_position;
  }

     limb_index = 0;
  do {
    printf("select one limb (enter limb_index or -1 to quit): ");
    int sr = scanf("%d", &limb_index);
    if (sr < 1) { printf("please enter number.\n"); continue; } 
    if (limb_index == -1) break;
  } while ((limb_index < 0) || (limb_index > 21));
            
  int position;
  if (limb_index >= 0)
  {
    printf("\nLimb selected: %d %s\n.Use +,- and *,/ to control the limb, ESC to quit.\n", limb_index, limb_names[limb_index]);
    position = positions[limb_index];
  }

  while (limb_index >= 0)
  {
    int chr = getch();
    if (chr == ESC_ASCII_VALUE)
    {
      break;
      limb_index = -1;
    }
    int change = 0;
    if (chr == '+') 
    {
	    position++;
	    change = 1;
    }
    else if (chr == '-') 
    {
	    position--;
	    change = 1;
    }
    else if (chr == '*') 
    {
	    position += 16;
	    change = 1;
    }
    else if (chr == '/') 
    {
	    position -= 16;
	    change = 1;
    }
    else if (chr == ' ')
    {
	    break;
    }


    if (change)
    {
	    printf("%05d\t", position);
                write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[limb_index], ADDR_MX_MOVING_SPEED, 50);
                if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                {
                  printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
                }
                else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
                {
                  printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
                }
            
                // Write goal position
                write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[limb_index], ADDR_MX_GOAL_POSITION, position);
                if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                {
                  printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
                }
                else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
                {
                  printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
                }
            
                  // Read present position
                  dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID[limb_index], ADDR_MX_PRESENT_POSITION);
                  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                  {
                    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
                  }
                  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
                  {
                    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
                  }

		  printf("%05d\n", dxl_present_position);
    }
  }
  } while(limb_index >= 0);

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
  printf("bye.\n");

  return 0;
}
