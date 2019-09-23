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
// *********     Sync Read and Sync Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000)
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
enum ControlMode { //当前的电机控制模式
	ModeTorqueControl = 0,
	ModeSpeedCtrlWithoutTrajectory = 1,
	ModePosCtrlWithoutTrajectorySingleTurn = 2,
	ModePosCtrlWithoutTrajectory = 3,
	ModePosCtrlWithTrajectory = 4,
	ModeSpeedCtrlWithTrajectory = 5,
	ModePulseWidthCtrl = 6
};


#define ADDR_PRO_OPERATE_MODE           11                 // Control table address is different in Dynamixel model
#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_GOAL_VELOCITY          600
#define ADDR_PRO_GOAL_TORQUE            602
#define ADDR_PRO_PRESENT_POSITION       611

#define ADDR_PRO_MAX_TORQUE       30
#define ADDR_PRO_MAX_VEL          32
#define ADDR_PRO_MAX_POS          36
#define ADDR_PRO_MIN_POS          40

// Data Byte Length
#define LEN_PRO_GOAL_POSITION            4
#define LEN_PRO_GOAL_VELOCITY            4
#define LEN_PRO_GOAL_TORQUE              2
#define LEN_PRO_GOAL_ACC                 4
#define LEN_PRO_PRESENT_POSITION         4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "COM4"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      -150             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      150              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

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

#define DXL_TEST_NUMBER    6
#define DXL_NUMBER    6
uint8_t dxl_ids[DXL_NUMBER]              = { 1,2,3,4,5,6 };
uint8_t homing_dir[DXL_NUMBER]           = { 0,1,0,0,0,0 };  // 寻找零位的初始方向,0=順時針，1=逆時針（面向电机出轴方向看）
int32_t dxl_present_position[DXL_NUMBER] = { 0,0,0,0,0,0 };              // Present position

int32_t dxl_max_position[DXL_NUMBER] = { 4096,1500,  1300,4096,2650,409600 };              // 位置上限
int32_t dxl_min_position[DXL_NUMBER] = { 0,   -500, -1300,   0, 350,     -409600 };              // 位置下限
int32_t dxl_max_velocity[DXL_NUMBER] = { 800, 800,    800,1200,1200,  1600 };              // 速度上限
int16_t dxl_max_torque[DXL_NUMBER] = { 4400, 4400,   4400,2200,2200,  2200 };              // 转矩上限
/*
int32_t dxl_max_position[DXL_NUMBER] = { 2000,3050,2700,800,400,400 };              // 位置上限
int32_t dxl_min_position[DXL_NUMBER] = { -800,2100,1100,-800,-400,-400 };              // 位置下限
int32_t dxl_max_velocity[DXL_NUMBER] = { 1200,1200,1200,800,800,800 };              // 速度上限
int16_t dxl_max_torque[DXL_NUMBER] = { 1800,1800,1800,1800,1000,1000 };              // 转矩上限
*/
int main() {
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
	touchideas_driver::PortHandler *portHandler = touchideas_driver::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	touchideas_driver::PacketHandler *packetHandler = touchideas_driver::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
	touchideas_driver::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION
	  + LEN_PRO_GOAL_VELOCITY+ LEN_PRO_GOAL_TORQUE+ LEN_PRO_GOAL_ACC);

  // Initialize Groupsyncread instance for Present Position
	touchideas_driver::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  int index = 0;
  CommErrorCode dxl_comm_result = CommTxFailed;             // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

#define STEPS_COUNT 6
  int32_t dxl_goal_position[STEPS_COUNT][DXL_NUMBER] = {
	  { 3350,   500, -400, 604,895,0 },
      { 3350,  836,  264,  604, 895,0 },
      { 3350,  836,  255,  604, 885,4096 },
      { 4090,   500, -400, 604,895,0 },
	  { 4090,  836,  264,  604, 895,0 },
	  { 4090,  836,  255,  604, 885,4096 }
  };         // Goal position
  uint16_t dxl_goal_torque[DXL_NUMBER] = {
   	  4400, 4400, 4400, 2200,2200,2200
  };         // Goal position
  uint16_t dxl_goal_acc[DXL_NUMBER] = {
	  3500, 2500, 3500, 4500,4500,30000
  };         // Goal position
  uint16_t dxl_max_acc[DXL_NUMBER] = {
	  3500, 2500, 3500, 4500,4500,30000
  };         // Goal position
  uint16_t dxl_max_vel[DXL_NUMBER] = {
	  3500, 2500, 3500, 4500,4500,30000
  };         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint8_t param_goal_position[20];

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

  // 设定各伺服的转矩上限
  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
	  while (1) {
		  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_ids[i], ADDR_PRO_MAX_TORQUE, dxl_max_torque[i], &dxl_error);
		  if (dxl_comm_result != CommSuccess) {
			  printf("DXL#%d max torque failed:%s\n", dxl_ids[i], packetHandler->getTxRxResult(dxl_comm_result));
		  }
		  else if (dxl_error != 0) {
			  printf("DXL#%d %s\n", dxl_ids[i], packetHandler->getRxPacketError(dxl_error));
			  break;
		  }
		  else {
			  printf("DXL#%d max torque: %d\n", dxl_ids[i], dxl_max_torque[i]);
			  break;
		  }
	  }
  }

  // 设定各伺服的速度上限
  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
	  while (1) {
		  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_ids[i], ADDR_PRO_MAX_VEL, dxl_max_velocity[i], &dxl_error);
		  if (dxl_comm_result != CommSuccess) {
			  printf("DXL#%d max velocity  failed:%s\n", dxl_ids[i], packetHandler->getTxRxResult(dxl_comm_result));
		  }
		  else if (dxl_error != 0) {
			  printf("DXL#%d %s\n", dxl_ids[i], packetHandler->getRxPacketError(dxl_error));
			  break;
		  }
		  else {
			  printf("DXL#%d max velocity: %d\n", dxl_ids[i], dxl_max_velocity[i]);
			  break;
		  }
	  }
  }

  // 使能各伺服为位置控制方式
  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
	  while (1) {
		  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_PRO_OPERATE_MODE, ModePosCtrlWithoutTrajectorySingleTurn, &dxl_error);
		  if (dxl_comm_result != CommSuccess) {
			  printf("DXL#%d operate mode failed:%s\n", dxl_ids[i], packetHandler->getTxRxResult(dxl_comm_result));
		  }
		  else if (dxl_error != 0) {
			  printf("DXL#%d %s\n", dxl_ids[i], packetHandler->getRxPacketError(dxl_error));
			  break;
		  }
		  else {
			  printf("DXL#%d has been successfully set operate mode \n", dxl_ids[i]);
			  break;
		  }
	  }
  }

  // 将一号到六号电机的读取项目添加到同步读取参数表，读取当前位置
  // Add parameter storage for Dynamixel#2 present position value
  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
	  dxl_addparam_result = groupSyncRead.addParam(dxl_ids[i]);
	  if (dxl_addparam_result != true) {
		  fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", dxl_ids[i]);
		  return 0;
	  }
  }
  // 使用同步读取命令来读取当前位置
  do {
	  dxl_comm_result = groupSyncRead.txRxPacket();
	  if (dxl_comm_result != CommSuccess)
		  printf("sync read failed: %s\n", packetHandler->getTxRxResult(dxl_comm_result));
	  else
		  break;
  } while (1);

  // 检查各电机的当前位置是否都读取成功
  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
	  dxl_getdata_result = groupSyncRead.isAvailable(dxl_ids[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
	  if (dxl_getdata_result != true) {
		  fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dxl_ids[i]);
		  //return 0;
	  }
  }

  // 读取接收到的各电机的当前位置值
  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
	  dxl_present_position[i] = groupSyncRead.getData(dxl_ids[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
	  // 根据电机的初始位置，调整上下限设定和运行区间设定
	  if ( dxl_present_position[i] > dxl_max_position[i] ) {
		  dxl_max_position[i] += 4096;
		  dxl_min_position[i] += 4096;
		  for(int k=0;k<STEPS_COUNT; k++)
		    dxl_goal_position[k][i] += 4096;
	  } else if (dxl_present_position[i] < dxl_min_position[i]) {
		  dxl_max_position[i] -= 4096;
		  dxl_min_position[i] -= 4096;
		  for (int k = 0; k<STEPS_COUNT; k++)
			  dxl_goal_position[k][i] -= 4096;
	  }
	  printf("[ID:%03d] GoalPos:%03d,%03d Max:%03d Min:%03d PresPos:%03d\n", dxl_ids[i], dxl_goal_position[0][i], dxl_goal_position[1][i], dxl_max_position[i], dxl_min_position[i],
		  dxl_present_position[i]);
  }

  // 设定各伺服的位置上限
  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
	  while (1) {
		  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_ids[i], ADDR_PRO_MAX_POS, dxl_max_position[i], &dxl_error);
		  if (dxl_comm_result != CommSuccess) {
			  printf("DXL#%d max pos failed:%s\n", dxl_ids[i], packetHandler->getTxRxResult(dxl_comm_result));
		  } else if (dxl_error != 0) {
			  printf("DXL#%d %s\n", dxl_ids[i], packetHandler->getRxPacketError(dxl_error));
			  break;
		  }
		  else {
			  printf("DXL#%d max pos: %d\n", dxl_ids[i], dxl_max_position[i]);
			  break;
		  }
	  }
  }
  // 设定各伺服的位置下限
  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
	  while (1) {
		  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_ids[i], ADDR_PRO_MIN_POS, dxl_min_position[i], &dxl_error);
		  if (dxl_comm_result != CommSuccess) {
			  printf("DXL#%d min pos failed:%s\n", dxl_ids[i], packetHandler->getTxRxResult(dxl_comm_result));
		  }
		  else if (dxl_error != 0) {
			  printf("DXL#%d %s\n", dxl_ids[i], packetHandler->getRxPacketError(dxl_error));
			  break;
		  }
		  else {
			  printf("DXL#%d min pos: %d\n", dxl_ids[i], dxl_min_position[i]);
			  break;
		  }
	  }
  }

  // 使能各伺服的力矩输出
  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	  while (1) {
		  if (dxl_comm_result != CommSuccess) {
			  printf("DXL#%d torque enable failed:%s\n", dxl_ids[i], packetHandler->getTxRxResult(dxl_comm_result));
		  }
		  else if (dxl_error != 0) {
			  printf("DXL#%d %s\n", dxl_ids[i], packetHandler->getRxPacketError(dxl_error));
			  break;
		  }
		  else {
			  printf("DXL#%d has been successfully torque enabled \n", dxl_ids[i]);
			  break;
		  }
	  }
  }
  /*
  // 驱动各电机归零
  for (int i = 0; i<4; i++) {
	  dxl_comm_result = packetHandler->homing(portHandler, dxl_ids[i], homing_dir[i], &dxl_error);
	  if (dxl_comm_result != COMM_SUCCESS) {
		  printf("DXL#%d homing aborted:%s\n", dxl_ids[i], packetHandler->getTxRxResult(dxl_comm_result));
		  return 0;
	  } else {
		  if (dxl_error != 0)
			  printf("DXL#%d %s\n", dxl_ids[i], packetHandler->getRxPacketError(dxl_error));
		  // 命令发送成功，读取状态，直到一个电机归零完毕，再进行下一电机的归零
		  while (1) {
			  uint8_t torque = 0;
			  dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, dxl_ids[i], ADDR_PRO_TORQUE_ENABLE, &torque, &dxl_error);
			  if (dxl_comm_result != COMM_SUCCESS) {
				  printf("DXL#%d homing read:%s\n", dxl_ids[i], packetHandler->getTxRxResult(dxl_comm_result));
				  packetHandler->printTxRxResult(dxl_comm_result);
			  } else if (!(torque & 0x2)) {
				  printf("DXL#%d has been successfully homed \n", dxl_ids[i]);
				  break;
			  }
		  }
	  }
  }*/


  bool goaled =false;
  int printCount = 0;

  while(1) {
	  Sleep(1000);
    //printf("Press any key to continue! (or press ESC to quit!)\n");
    //if (getch() == ESC_ASCII_VALUE)
      //break;

	// 使用同步读取命令来读取当前位置  == 读取当前的精确位置，因为前面读取的位置可能不是最终位置
	dxl_comm_result = groupSyncRead.txRxPacket();
	if (dxl_comm_result != CommSuccess)
		printf("sync read failed: %s\n", packetHandler->getTxRxResult(dxl_comm_result));

	// 检查各电机的当前位置是否都读取成功
	for (int i = 0; i < DXL_TEST_NUMBER; i++) {
		dxl_getdata_result = groupSyncRead.isAvailable(dxl_ids[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
		if (dxl_getdata_result != true) {
			fprintf(stderr, "[%03d] groupSyncRead getdata failed", dxl_ids[i]);
			//return 0;
		}
	}

	// 读取接收到的各电机的当前位置值
	for (int i = 0; i < DXL_TEST_NUMBER; i++) {
		dxl_present_position[i] = groupSyncRead.getData(dxl_ids[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
	}

	// 将目标地址设定写入到同步写入参数表中
	for (int i = 0; i < DXL_TEST_NUMBER; i++) {
		// 将目标地址值装配为字节数组
		int32_t s = abs(dxl_goal_position[index][i] - dxl_present_position[i]);
		if (s >= DXL_MOVING_STATUS_THRESHOLD) {
			touchideas_driver::PacketHandler::packetWord(&param_goal_position[0], dxl_goal_position[index][i]);
			// 将目标速度值装配为字节数组，这里固定每个动作时间为4秒，因为假设是一个完整的梯形，所以这个计算式成立的条件是：vel < (acc*t)/2 
			// 已知位移，加速度和运行时间，求取最大（巡航）速度 v = (at<+->sqrt(a*a*t*t-4sa))/2;因为有约束 vel < (acc*t)/2 ，所以只能去减号这个值
			int32_t t = 4;
			int32_t delta = dxl_goal_acc[i] * t*dxl_goal_acc[i] * t - 4 * s*dxl_goal_acc[i];
			if (delta < 0) {
				fprintf(stderr, "[%03d] Trajectory invalide s=%d t=%d a=%d delta=%d", dxl_ids[i], s, t, dxl_goal_acc[i], delta);
				return 0;
			}
			int32_t  vel = (dxl_goal_acc[i] * t - sqrt(delta)) / 2;
			if (abs(vel) > 7000)
				vel = 7000;
			if (abs(vel) < 20)
				vel = 20;
			if (vel > 0)
				vel = -vel;  //因为我们的演示都是停位速度为零，所以肯定有后半段
			touchideas_driver::PacketHandler::packetWord(&param_goal_position[4], vel);

			// 将目标力矩值装配为字节数组
			touchideas_driver::PacketHandler::packetHalfWord(&param_goal_position[8], dxl_goal_torque[i]);

			// 将目标加速度值装配为字节数组
			touchideas_driver::PacketHandler::packetWord(&param_goal_position[10], dxl_goal_acc[i]);
			dxl_addparam_result = groupSyncWrite.addParam(dxl_ids[i], param_goal_position);
			printf("[ID:%03d]S:%d,Go:%03d,V:%03d,T:%03d,A:%03d  ", dxl_ids[i], s, dxl_goal_position[index][i], vel, dxl_goal_torque[i], dxl_goal_acc[i]);

			if (dxl_addparam_result != true) {
				fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", dxl_ids[i]);
				return 0;
			}
		} else
			printf("[%03d]S:%d,Go:%03d  ", dxl_ids[i], s, dxl_goal_position[index][i]);
	}
	printf("\n");

	// 发送同步写入指令
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != CommSuccess)
		printf("sync write failed: %s\n", packetHandler->getTxRxResult(dxl_comm_result));

    // 清空同步写入参数表
    groupSyncWrite.clearParam();

    do {
      // 使用同步读取命令来读取当前位置
      dxl_comm_result = groupSyncRead.txRxPacket();
      if (dxl_comm_result != CommSuccess)
		  printf("sync read failed: %s\n", packetHandler->getTxRxResult(dxl_comm_result));

      // 检查各电机的当前位置是否都读取成功
	  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
		  dxl_getdata_result = groupSyncRead.isAvailable(dxl_ids[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
		  if (dxl_getdata_result != true) {
			  fprintf(stderr, "[%03d] groupSyncRead getdata failed", dxl_ids[i]);
			  //return 0;
		  }
	  }

	  // 读取接收到的各电机的当前位置值
	  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
		  dxl_present_position[i] = groupSyncRead.getData(dxl_ids[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
		  if (printCount > 50) {
			  printf("[%03d]Go:%03d Pos:%03d  ", dxl_ids[i], dxl_goal_position[index][i],
				  dxl_present_position[i]);
		  }
	  }
	  if (printCount > 50) {
		  printCount = 0;
		  printf("\n");
	  } else
		printCount++;

	  goaled = true;
	  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
		  if ((abs(dxl_goal_position[index][i] - dxl_present_position[i]) > DXL_MOVING_STATUS_THRESHOLD))
			  goaled = false;
	  }
	  if (goaled) {
		  printf("Goal:");
		  for (int i = 0; i < DXL_TEST_NUMBER; i++) {
			  printf("[%03d]Go:%03d Pos:%03d   ", dxl_ids[i], dxl_goal_position[index][i],
				  dxl_present_position[i]);
		  }
		  printf("\n");
	  }
	}while( !goaled );

    // 改变目标位置
	index++;
	if (index >= STEPS_COUNT)
		index = 0;
  }

  // 关闭伺服电机力矩输出
  for (int i = 0; i < DXL_NUMBER; i++) {
	  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_ids[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	  if (dxl_comm_result != CommSuccess) {
		  fprintf(stderr, "[ID:%03d] %s", dxl_ids[i], packetHandler->getRxPacketError(dxl_comm_result));

	  } else if (dxl_error != 0) {
		  fprintf(stderr, "[ID:%03d] %s", dxl_ids[i], packetHandler->getRxPacketError(dxl_error));
	  }
  }

  // 关闭串口
  portHandler->closePort();

  return 0;
}
