/* Authors: Zhwei123 */

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include "../../include/dynamixel_sdk/port_handler.h"

#ifdef __linux__
  #include "../../include/dynamixel_sdk_linux/port_handler_linux.h"
#endif

#if defined(_WIN32) || defined(_WIN64)
  #include "../../include/dynamixel_sdk_windows/port_handler_windows.h"
#endif

using namespace TouchIdeas485;

const double PortHandler::_SwitchDelay =0.4;  // 串口收发中间的切换延时（包含从机的响应时间）
const double PortHandler::_BufDelay =0.2;  // 串口缓存区到实际发送的延时

double PortHandler::setPacketTimeout(uint16_t packet_length, bool needRecv) {
    packet_start_time_ = getCurrentTime();
    packet_timeout_ = (tx_time_per_byte_ * (packet_length+_LatencyTimer)) + _BufDelay
            + (needRecv?_SwitchDelay:0.) ;
    return packet_timeout_;
}

void PortHandler::setPacketTimeout(double msec) {
    packet_start_time_ = getCurrentTime();
    packet_timeout_ = msec;
}

bool PortHandler::isPacketTimeout() {
    if (getTimeSinceStart() > packet_timeout_) {
        packet_timeout_ = 0;
        return true;
    }
    return false;
}

double PortHandler::getTimeSinceStart() {
    double time;

    time = getCurrentTime() - packet_start_time_;
    if (time < 0.0)
        packet_start_time_ = getCurrentTime();

    return time;
}

// 创建一个串口句柄，我们通过它与 RS485 进行通讯
PortHandler *PortHandler::getPortHandler(const char *port_name) {
#ifdef __linux__
    return reinterpret_cast<PortHandler *>(new PortHandlerLinux(port_name));
#endif

#if defined(_WIN32) || defined(_WIN64)
    return reinterpret_cast<PortHandler *>(new PortHandlerWindows(port_name));
#endif
}
