/* Authors: Zhwei123 */

#ifndef TOUCHIDEAS_SDK_DYNAMIXELSDK_H_
#define TOUCHIDEAS_SDK_DYNAMIXELSDK_H_

#include "dynamixel_sdk/group_bulk_read.h"
#include "dynamixel_sdk/group_bulk_write.h"
#include "dynamixel_sdk/group_sync_read.h"
#include "dynamixel_sdk/group_sync_write.h"
#include "dynamixel_sdk/protocol1_packet_handler.h"
#include "dynamixel_sdk/protocol2_packet_handler.h"

#ifdef __linux__
  #include "dynamixel_sdk_linux/port_handler_linux.h"
#endif

#if defined(_WIN32) || defined(_WIN64)
  #include "dynamixel_sdk_windows/port_handler_windows.h"
#endif

#endif /* TOUCHIDEAS_SDK_DYNAMIXELSDK_H_ */
