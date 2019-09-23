/* Authors: Zhwei123 */

#ifndef TOUCHIDEAS_SDK_WINDOWS_PORTHANDLERWINDOWS_H_
#define TOUCHIDEAS_SDK_WINDOWS_PORTHANDLERWINDOWS_H_

#define    WIN32_LEAN_AND_MEAN   //去除一些不常用的
#include <WinSock2.h>
#include <Windows.h>

#include "../dynamixel_sdk/port_handler.h"

namespace TouchIdeas485 {
class WINDECLSPEC PortHandlerWindows : public PortHandler {
public:
    virtual ~PortHandlerWindows() { closePort(); }
    friend class PortHandler;

    bool openPort();   //打开串口
    void closePort();  //关闭串口
    void clearPort();  //清空串口缓存区

    // 设定和获取串口波特率
    bool setBaudRate(const uint32_t baudrate);
    uint32_t getBaudRate();

    // 获取串口接收缓存区可用字节
    int getBytesAvailable();

    // 读写串口接收和发送缓存区
    int readPort(uint8_t *packet, int length);
    int writePort(uint8_t *packet, int length);

private:
    PortHandlerWindows(const char *port_name);

    HANDLE serial_handle_;
    LARGE_INTEGER freq_, counter_;

    bool setupPort(uint32_t baudrate);

    double getCurrentTime();
    double getTimeSinceStart();
};

}


#endif /* DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_LINUX_PORTHANDLERWINDOWS_H_ */
