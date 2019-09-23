/* Authors: Zhwei123 */

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include "../../include/dynamixel_sdk_windows/port_handler_windows.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

using namespace TouchIdeas485;

PortHandlerWindows::PortHandlerWindows(const char *port_name)
  : serial_handle_(INVALID_HANDLE_VALUE) {
  char buffer[15];
  sprintf_s(buffer, sizeof(buffer), "\\\\.\\%s", port_name);
  setPortName(buffer);
}

bool PortHandlerWindows::openPort() {
    _isOpen =setBaudRate(baudrate_);
  return _isOpen;
}

void PortHandlerWindows::closePort() {
  if (serial_handle_ != INVALID_HANDLE_VALUE) {
    CloseHandle(serial_handle_);
    serial_handle_ = INVALID_HANDLE_VALUE;
    _isOpen =false;
  }
}

void PortHandlerWindows::clearPort() {
  PurgeComm(serial_handle_, PURGE_RXABORT | PURGE_RXCLEAR);
}

bool PortHandlerWindows::setBaudRate(const uint32_t baudrate) {
  closePort();

  baudrate_ = baudrate;
  return setupPort(baudrate);
}

uint32_t PortHandlerWindows::getBaudRate() {
  return baudrate_;
}

int PortHandlerWindows::getBytesAvailable() {
  DWORD retbyte = 2;
  BOOL res = DeviceIoControl(serial_handle_, GENERIC_READ | GENERIC_WRITE, nullptr, 0, nullptr, 0, &retbyte, static_cast<LPOVERLAPPED>(nullptr));

  printf("%d", static_cast<int>(res));
  return static_cast<int>(retbyte);
}

int PortHandlerWindows::readPort(uint8_t *packet, int length) {
  DWORD dwRead = 0;

  if (ReadFile(serial_handle_, packet, static_cast<DWORD>(length), &dwRead, nullptr) == FALSE)
    return -1;

  return static_cast<int>(dwRead);
}

int PortHandlerWindows::writePort(uint8_t *packet, int length) {
  DWORD dwWrite = 0;

  if (WriteFile(serial_handle_, packet, static_cast<DWORD>(length), &dwWrite, nullptr) == FALSE)
    return -1;

  return static_cast<int>(dwWrite);
}

double PortHandlerWindows::getCurrentTime() {
  QueryPerformanceCounter(&counter_);
  QueryPerformanceFrequency(&freq_);
  return static_cast<double>(counter_.QuadPart) / static_cast<double>(freq_.QuadPart) * 1000.0;
}

bool PortHandlerWindows::setupPort(uint32_t baudrate) {
  DCB dcb;
  COMMTIMEOUTS timeouts;
  DWORD dwError;

  closePort();

  serial_handle_ = CreateFileA(port_name_, GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (serial_handle_ == INVALID_HANDLE_VALUE) {
    printf("[PortHandlerWindows::SetupPort] Error opening serial port!\n");
    return false;
  }

  dcb.DCBlength = sizeof(DCB);
  if (GetCommState(serial_handle_, &dcb) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  // Set baudrate
  DWORD baudRate =1000000;
  //if ( baudrate ==1000000 )
  //    baudRate = 1433333;
  dcb.BaudRate = static_cast<DWORD>(baudRate);
  dcb.ByteSize = 8;                    // Data bit = 8bit
  dcb.Parity = NOPARITY;             // No parity
  dcb.StopBits = ONESTOPBIT;           // Stop bit = 1
  dcb.fParity = NOPARITY;             // No Parity check
  dcb.fBinary = 1;                    // Binary mode
  dcb.fNull = 0;                    // Get Null byte
  dcb.fAbortOnError = 0;
  dcb.fErrorChar = 0;
  // Not using XOn/XOff
  dcb.fOutX = 0;
  dcb.fInX = 0;
  // Not using H/W flow control
  dcb.fDtrControl = DTR_CONTROL_DISABLE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;
  dcb.fDsrSensitivity = 0;
  dcb.fOutxDsrFlow = 0;
  dcb.fOutxCtsFlow = 0;

  if (SetCommState(serial_handle_, &dcb) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  if (SetCommMask(serial_handle_, 0) == FALSE) // Not using Comm event
    goto DXL_HAL_OPEN_ERROR;
  if (SetupComm(serial_handle_, 4096, 4096) == FALSE) // Buffer size (Rx,Tx)
    goto DXL_HAL_OPEN_ERROR;
  if (PurgeComm(serial_handle_, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR) == FALSE) // Clear buffer
    goto DXL_HAL_OPEN_ERROR;
  if (ClearCommError(serial_handle_, &dwError, nullptr) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  if (GetCommTimeouts(serial_handle_, &timeouts) == FALSE)
    goto DXL_HAL_OPEN_ERROR;
  // Timeout (Not using timeout)
  // Immediatly return
  timeouts.ReadIntervalTimeout = 0;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.ReadTotalTimeoutConstant = 1; // must not be zero.
  timeouts.WriteTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 0;
  if (SetCommTimeouts(serial_handle_, &timeouts) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  tx_time_per_byte_ = (1000.0 / static_cast<double>(baudrate_)) * 10.0;
  return true;

DXL_HAL_OPEN_ERROR:
  closePort();
  return false;
}
