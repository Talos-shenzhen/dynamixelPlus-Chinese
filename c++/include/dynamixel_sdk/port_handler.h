/* Authors: Zhwei123 */

#ifndef TOUCHIDEAS_SDK_PORTHANDLER_H_
#define TOUCHIDEAS_SDK_PORTHANDLER_H_

#ifdef __linux__
#define WINDECLSPEC
#elif defined(_WIN32) || defined(_WIN64)
#ifdef WINDLLEXPORT
#define WINDECLSPEC __declspec(dllexport)
#else
//#define WINDECLSPEC __declspec(dllimport)
#define WINDECLSPEC
#endif
#endif

#include <stdint.h>
#include <string>

namespace TouchIdeas485 {

class WINDECLSPEC PortHandler {
public:
    static const int DEFAULT_BAUDRATE_ = 1000000;    // 我们使用的默认波特率
    static const int _LatencyTimer = 16; // msec (USB latency timer)
    static const double _SwitchDelay;  // 串口收发中间的切换延时（包含从机的响应时间）
    static const double _BufDelay;  // 串口缓存区到实际发送的延时
    // 我们实际使用时，都是通过这个静态函数来申请一个串口句柄
    static PortHandler *getPortHandler(const char *port_name);

    PortHandler(void) : baudrate_(DEFAULT_BAUDRATE_), packet_start_time_(0.0),
        packet_timeout_(0.0), tx_time_per_byte_(0.0) {
    }
    virtual ~PortHandler() { }

    virtual bool openPort() = 0;   //打开串口
    virtual void closePort() = 0;  //关闭串口
    virtual void clearPort() = 0;  //清空串口缓存区

    virtual bool isOpen(void) const { return _isOpen; } // 判断串口是否打开

    // 设定和获取串口设备名称
    virtual void setPortName(const char* port_name) {
#ifdef __linux__
        strcpy(port_name_, sizeof(port_name_), port_name);
#else
        strcpy_s(port_name_, sizeof(port_name_), port_name);
#endif
    }
    virtual const char* getPortName() const {
        return port_name_;
    }

    // 设定和获取串口波特率
    virtual bool setBaudRate(const uint32_t baudrate) = 0;
    virtual uint32_t getBaudRate() = 0;

    // 获取串口接收缓存区可用字节
    virtual int getBytesAvailable() = 0;

    // 读写串口接收和发送缓存区
    virtual int readPort(uint8_t *packet, int length) = 0;
    virtual int writePort(uint8_t *packet, int length) = 0;

    // 设定串口接收超时，一种是指定接收数据报的长度；另一种是指定毫秒时间
    virtual double setPacketTimeout(uint16_t packet_length, bool needRecv=true);
    virtual void setPacketTimeout(double msec);
    virtual bool isPacketTimeout();   // 检查接收等待是否超时

    bool isUsing(void) const { return is_using_; }
    void setUsing(bool used) { is_using_ =used; }

protected:
    virtual double getCurrentTime()=0;
    double getTimeSinceStart();

    bool   is_using_ = false;
    bool   _isOpen =false;

    uint32_t baudrate_;
    char port_name_[30];

    double packet_start_time_;
    double packet_timeout_;
    double tx_time_per_byte_;
};

}


#endif /* TOUCHIDEAS_SDK_PORTHANDLER_H_ */
