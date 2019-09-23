/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Bulk Read and Bulk Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 and 2 / Baudnum : 3 (Baudrate : 1000000)
//

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_LED_RED                563
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611

// Data Byte Length
#define LEN_PRO_LED_RED                 1
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting

#define BAUDRATE                        1000000
#define DEVICENAME                      "COM7"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      -150             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

int getch() {
#ifdef __linux__
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

int kbhit(void) {
#ifdef __linux__
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

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main() {
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupBulkWrite instance
  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint8_t dxl_led_value[2] = {0x00, 0xFF};        // Dynamixel LED value for write
  uint8_t param_goal_position[4];
  int32_t dxl1_present_position = 0;              // Present position

#define DXL_NUMBER    6
  uint8_t dxl_ids[DXL_NUMBER] = {3,4,5,6,7,8};
  uint8_t dxl_led_value_read[DXL_NUMBER];         // Dynamixel LED value for read

  // 打开串口
  if (portHandler->openPort()) {
    printf("Succeeded to open the port!\n");
  } else {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // 设定串口波特率
  if (portHandler->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n");
  } else {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // 使能各伺服的力矩输出
  for (int i = 0; i < DXL_NUMBER; i++) {
	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS) {
		  packetHandler->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error != 0) {
		  packetHandler->printRxPacketError(dxl_error);
	  }
	  else {
		  printf("DXL#%d has been successfully connected \n", dxl_ids[i]);
	  }
  }

  // 为一号电机的块读取操作添加参数，读取当前位置
  dxl_addparam_result = groupBulkRead.addParam(dxl_ids[0], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
  if (dxl_addparam_result != true) {
    fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed\n", dxl_ids[0]);
    return 0;
  }

  // 为二号到六号电机的块读取操作添加参数，读取当前 LED 显示值
  for (int i = 1; i < DXL_NUMBER; i++) {
	  dxl_addparam_result = groupBulkRead.addParam(dxl_ids[i], ADDR_PRO_LED_RED, LEN_PRO_LED_RED);
	  if (dxl_addparam_result != true) {
		  fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed\n", dxl_ids[i]);
		  return 0;
	  }
  }

  while(1) {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // 将要发送的目标位置命令封装到字节数组中
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]));

    // 为一号电机添加块写入参数，目标位置
    dxl_addparam_result = groupBulkWrite.addParam(dxl_ids[0], ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION, param_goal_position);
    if (dxl_addparam_result != true) {
      fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed\n", dxl_ids[0]);
      return 0;
    }

    // 为二号到六号电机添加写入参数，LED 值
	for (int i = 1; i < DXL_NUMBER; i++) {
		dxl_addparam_result = groupBulkWrite.addParam(dxl_ids[i], ADDR_PRO_LED_RED, LEN_PRO_LED_RED, &dxl_led_value[index]);
		if (dxl_addparam_result != true) {
			fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed\n", dxl_ids[i]);
			return 0;
		}
	}

	// 执行块写入命令
    dxl_comm_result = groupBulkWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

    // 清空块写入参数表
    groupBulkWrite.clearParam();

    do {
      // 执行块读取命令
      dxl_comm_result = groupBulkRead.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

      // 看看我们是否读取到了一号电机的相关数据
      dxl_getdata_result = groupBulkRead.isAvailable(dxl_ids[0], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
      if (dxl_getdata_result != true) {
        fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", dxl_ids[0]);
     //   return 0;
      }

	  // 看看我们是否读取到了二号到六号电机的相关数据
	  for (int i = 1; i < DXL_NUMBER; i++) {
		  dxl_getdata_result = groupBulkRead.isAvailable(dxl_ids[i], ADDR_PRO_LED_RED, LEN_PRO_LED_RED);
		  if (dxl_getdata_result != true) {
			  fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", dxl_ids[i]);
			  // return 0;
		  }
	  }

	  // 读取一号电机的位置值
      dxl1_present_position = groupBulkRead.getData(dxl_ids[0], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

      // 读取二号到六号电机的 LED 值
	  for (int i = 1; i < DXL_NUMBER; i++) {
		  dxl_led_value_read[i] = groupBulkRead.getData(dxl_ids[i], ADDR_PRO_LED_RED, LEN_PRO_LED_RED);
	  }

	  // 打印读取到的结果
	  printf("[ID:%03d] Present Position : %d \t [ID:", dxl_ids[0], dxl1_present_position);
	  for (int i = 1; i < DXL_NUMBER; i++) {
		  if (i < DXL_NUMBER - 1)
			  printf("%03d,", dxl_ids[i]);
		  else
			  printf("%03d] LED Value: ", dxl_ids[i]);
	  }
	  for (int i = 1; i < DXL_NUMBER; i++) {
		  if (i < DXL_NUMBER - 1)
			  printf("%d,", dxl_led_value_read[i]);
		  else
			  printf("%d\n", dxl_led_value_read[i]);
	  }
    }while(abs(dxl_goal_position[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD);

    // Change goal position
    if (index == 0) {
      index = 1;
    } else {
      index = 0;
    }
  }

  // 关闭伺服电机力矩输出
  for (int i = 0; i < DXL_NUMBER; i++) {
	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS) {
		  packetHandler->printTxRxResult(dxl_comm_result);
	  }
	  else if (dxl_error != 0) {
		  packetHandler->printRxPacketError(dxl_error);
	  }
  }

  // 关闭串口
  portHandler->closePort();

ExitMain:
  return 0;
}
