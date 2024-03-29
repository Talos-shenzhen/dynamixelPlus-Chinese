﻿/*******************************************************************************
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

/* Author: zerom, Ryu Woon Jung (Leon) */

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include "../../include/dynamixel_sdk/packet_handler.h"
#include "../../include/dynamixel_sdk/protocol1_packet_handler.h"
#include "../../include/dynamixel_sdk/protocol2_packet_handler.h"

using namespace TouchIdeas485;

// 根据要求的版本，创建对应的 Dynammixel 协议对象
PacketHandler *PacketHandler::getPacketHandler(double protocol_version) {
	int ver = (int)(protocol_version * 10000);
  if (ver == 10000 ) {
    return (PacketHandler *)(Protocol1PacketHandler::getInstance());
  } else if (ver == 20000 ) {
    return (PacketHandler *)(Protocol2PacketHandler::getInstance());
  }

  return (PacketHandler *)(Protocol2PacketHandler::getInstance());
}

CommErrorCode PacketHandler::write1ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint8_t data) {
    uint8_t data_write[1] = { data };
    return writeTxOnly(port, id, address, 1, data_write);
}

CommErrorCode PacketHandler::write1ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint8_t data, uint8_t *error) {
  uint8_t data_write[1] = { data };
  return writeTxRx(port, id, address, 1, data_write, error);
}

CommErrorCode PacketHandler::write2ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t data) {
  uint8_t data_write[2];
  packetHalfWord(data_write, data);
  return writeTxOnly(port, id, address, 2, data_write);
}

CommErrorCode PacketHandler::write2ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t data, uint8_t *error) {
    uint8_t data_write[2];
    packetHalfWord(data_write, data);
    return writeTxRx(port, id, address, 2, data_write, error);
}

CommErrorCode PacketHandler::write4ByteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint32_t data) {
  uint8_t data_write[4];
  packetWord(data_write, data);
  return writeTxOnly(port, id, address, 4, data_write);
}

CommErrorCode PacketHandler::write4ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint32_t data, uint8_t *error) {
    uint8_t data_write[4];
    packetWord(data_write, data);
    return writeTxRx(port, id, address, 4, data_write, error);
}

CommErrorCode PacketHandler::read1ByteTx(PortHandler *port, uint8_t id, uint16_t address) {
  return readTx(port, id, address, 1);
}

CommErrorCode PacketHandler::read1ByteRx(PortHandler *port, uint8_t *data, uint8_t *error) {
  uint8_t data_read[1] = {0};
  CommErrorCode result = readRx(port, 1, data_read, error);
  if (result == CommSuccess)
    *data = data_read[0];
  return result;
}

CommErrorCode PacketHandler::read1ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint8_t *data, uint8_t *error) {
  uint8_t data_read[1] = {0};
  uint16_t len =1;
  CommErrorCode result = readTxRx(port, id, address, &len, data_read, error);
  if (result == CommSuccess)
    *data = data_read[0];
  return result;
}

CommErrorCode PacketHandler::read2ByteTx(PortHandler *port, uint8_t id, uint16_t address) {
  return readTx(port, id, address, 2);
}

CommErrorCode PacketHandler::read2ByteRx(PortHandler *port, uint16_t *data, uint8_t *error) {
  uint8_t data_read[2] = {0};
  CommErrorCode result = readRx(port, 2, data_read, error);
  if (result == CommSuccess)
    *data = getHalfWord(&data_read[0]);
  return result;
}

CommErrorCode PacketHandler::read2ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t *data, uint8_t *error) {
  uint8_t data_read[2] = {0};
  uint16_t len =2;
  CommErrorCode result = readTxRx(port, id, address, &len, data_read, error);
  if (result == CommSuccess)
      *data = getHalfWord(data_read);
  return result;
}

CommErrorCode PacketHandler::read4ByteTx(PortHandler *port, uint8_t id, uint16_t address) {
  return readTx(port, id, address, 4);
}

CommErrorCode PacketHandler::read4ByteRx(PortHandler *port, uint32_t *data, uint8_t *error) {
  uint8_t data_read[4] = {0};
  CommErrorCode result = readRx(port, 4, data_read, error);
  if (result == CommSuccess)
    *data = getWord(data_read);
  return result;
}

CommErrorCode PacketHandler::read4ByteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint32_t *data, uint8_t *error) {
  uint8_t data_read[4] = {0};
  uint16_t len =4;
  CommErrorCode result = readTxRx(port, id, address, &len, data_read, error);
  if (result == CommSuccess)
    *data = getWord(data_read);
  return result;
}
