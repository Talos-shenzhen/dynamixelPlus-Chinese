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

/* Author: zerom, Ryu Woon Jung (Leon) */

#if defined(__linux__)
#include "protocol1_packet_handler.h"
#elif defined(__APPLE__)
#include "protocol1_packet_handler.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "../../include/dynamixel_sdk/protocol1_packet_handler.h"
#elif defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__)
#include "../../include/dynamixel_sdk/protocol1_packet_handler.h"
#endif

#include <string.h>
#include <stdlib.h>

#define TXPACKET_MAX_LEN    (250)
#define RXPACKET_MAX_LEN    (250)

///////////////// for Protocol 1.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_ID                  2
#define PKT_LENGTH              3
#define PKT_INSTRUCTION         4
#define PKT_ERROR               4
#define PKT_PARAMETER0          5

///////////////// Protocol 1.0 Error bit /////////////////
#define ERRBIT_VOLTAGE          1       // Supplied voltage is out of the range (operating volatage set in the control table)
#define ERRBIT_ANGLE            2       // Goal position is written out of the range (from CW angle limit to CCW angle limit)
#define ERRBIT_OVERHEAT         4       // Temperature is out of the range (operating temperature set in the control table)
#define ERRBIT_RANGE            8       // Command(setting value) is out of the range for use.
#define ERRBIT_CHECKSUM         16      // Instruction packet checksum is incorrect.
#define ERRBIT_OVERLOAD         32      // The current load cannot be controlled by the set torque.
#define ERRBIT_INSTRUCTION      64      // Undefined instruction or delivering the action command without the reg_write command.

using namespace TouchIdeas485;

Protocol1PacketHandler::Protocol1PacketHandler() { }

Protocol1PacketHandler *Protocol1PacketHandler::getInstance() {
    return new Protocol1PacketHandler();
}

const char *Protocol1PacketHandler::getTxRxResult(CommErrorCode result) {
  switch(result) {
    case CommSuccess:
      return "[TxRxResult] Communication success.";

    case CommPortBusy:
      return "[TxRxResult] Port is in use!";

    case CommTxFailed:
      return "[TxRxResult] Failed transmit instruction packet!";

    case CommRxFailed:
      return "[TxRxResult] Failed get status packet from device!";

    case CommTxError:
      return "[TxRxResult] Incorrect instruction packet!";

    case CommRxWaiting:
      return "[TxRxResult] Now recieving status packet!";

    case CommRxTimeout:
      return "[TxRxResult] There is no status packet!";

    case CommRxCorrupt:
      return "[TxRxResult] Incorrect status packet!";

    case CommNotImplement:
      return "[TxRxResult] Protocol does not support This function!";

    default:
      return "";
  }
}

const char *Protocol1PacketHandler::getRxPacketError(uint8_t error) {
  if (error & ERRBIT_VOLTAGE)
    return "[RxPacketError] Input voltage error!";

  if (error & ERRBIT_ANGLE)
    return "[RxPacketError] Angle limit error!";

  if (error & ERRBIT_OVERHEAT)
    return "[RxPacketError] Overheat error!";

  if (error & ERRBIT_RANGE)
    return "[RxPacketError] Out of range error!";

  if (error & ERRBIT_CHECKSUM)
    return "[RxPacketError] Checksum error!";

  if (error & ERRBIT_OVERLOAD)
    return "[RxPacketError] Overload error!";

  if (error & ERRBIT_INSTRUCTION)
    return "[RxPacketError] Instruction code error!";

  return "";
}

CommErrorCode Protocol1PacketHandler::txPacket(PortHandler *port, uint8_t *txpacket) {
  uint8_t checksum               = 0;
  uint8_t total_packet_length    = txpacket[PKT_LENGTH] + 4; // 4: HEADER0 HEADER1 ID LENGTH
  uint8_t written_packet_length  = 0;

  if ( port->isUsing() )
    return CommPortBusy;
  port->setUsing(true);

  // check max packet length
  if (total_packet_length > TXPACKET_MAX_LEN) {
    port->setUsing(false);
    return CommTxError;
  }

  // make packet header
  txpacket[PKT_HEADER0]   = 0xFF;
  txpacket[PKT_HEADER1]   = 0xFF;

  // add a checksum to the packet
  for (int idx = 2; idx < total_packet_length - 1; idx++)   // except header, checksum
    checksum += txpacket[idx];
  txpacket[total_packet_length - 1] = ~checksum;

  // tx packet
  port->clearPort();
  written_packet_length = port->writePort(txpacket, total_packet_length);
  if (total_packet_length != written_packet_length) {
    port->setUsing(false);
    return CommTxFailed;
  }

  return CommSuccess;
}

CommErrorCode Protocol1PacketHandler::rxPacket(PortHandler *port, uint8_t *rxpacket) {
  CommErrorCode     result         = CommTxFailed;

  uint8_t checksum       = 0;
  uint8_t rx_length      = 0;
  uint8_t wait_length    = 6;    // minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

  while(true) {
    rx_length += port->readPort(&rxpacket[rx_length], wait_length - rx_length);
    if (rx_length >= wait_length) {
      uint8_t idx = 0;

      // find packet header
      for (idx = 0; idx < (rx_length - 1); idx++) {
        if (rxpacket[idx] == 0xFF && rxpacket[idx+1] == 0xFF)
          break;
      }

      if (idx == 0)  { // found at the beginning of the packet
        if (rxpacket[PKT_ID] > 0xFD ||                  // unavailable ID
           rxpacket[PKT_LENGTH] > RXPACKET_MAX_LEN ||   // unavailable Length
           rxpacket[PKT_ERROR] >= 0x64)  {               // unavailable Error
            // remove the first byte in the packet
            for (uint8_t s = 0; s < rx_length - 1; s++)
              rxpacket[s] = rxpacket[1 + s];
            //memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
            rx_length -= 1;
            continue;
        }

        // re-calculate the exact length of the rx packet
        if (wait_length != rxpacket[PKT_LENGTH] + PKT_LENGTH + 1) {
          wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1;
          continue;
        }

        if (rx_length < wait_length) {
          // check timeout
          if (port->isPacketTimeout() == true) {
            if (rx_length == 0) {
              result = CommRxTimeout;
            } else {
              result = CommRxCorrupt;
            }
            break;
          } else {
            continue;
          }
        }

        // calculate checksum
        for (int i = 2; i < wait_length - 1; i++)   // except header, checksum
          checksum += rxpacket[i];
        checksum = ~checksum;

        // verify checksum
        if (rxpacket[wait_length - 1] == checksum) {
          result = CommSuccess;
        } else {
          result = CommRxCorrupt;
        }
        break;
      } else {
        // remove unnecessary packets
        for (uint8_t s = 0; s < rx_length - idx; s++)
          rxpacket[s] = rxpacket[idx + s];
        //memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
        rx_length -= idx;
      }
    } else {
      // check timeout
      if (port->isPacketTimeout() == true) {
        if (rx_length == 0) {
          result = CommRxTimeout;
        } else {
          result = CommRxCorrupt;
        }
        break;
      }
    }
  }
  port->setUsing(false);

  return result;
}

// NOT for BulkRead instruction
CommErrorCode Protocol1PacketHandler::txRxPacket(PortHandler *port, uint8_t *txpacket, uint8_t *rxpacket, uint8_t *error) {
    // tx packet
    CommErrorCode result = txPacket(port, txpacket);
    if (result != CommSuccess)
      return result;

    // (ID == Broadcast ID && NOT BulkRead) == no need to wait for status packet
    // (Instruction == action) == no need to wait for status packet
    if ((txpacket[PKT_ID] == _BroadCastId && txpacket[PKT_INSTRUCTION] != InstBulkRead) ||
       (txpacket[PKT_INSTRUCTION] == InstAction)) {
        port->setUsing(false);
        return result;
    }

    // set packet timeout
    if (txpacket[PKT_INSTRUCTION] == InstRead) {
      port->setPacketTimeout(static_cast<uint16_t>(txpacket[PKT_PARAMETER0+1] + 6));
    } else {
      port->setPacketTimeout(static_cast<uint16_t>(6));
    }

    // rx packet
    result = rxPacket(port, rxpacket);
    // check txpacket ID == rxpacket ID
    if (txpacket[PKT_ID] != rxpacket[PKT_ID])
      result = rxPacket(port, rxpacket);

    if (result == CommSuccess && txpacket[PKT_ID] != _BroadCastId) {
      if (error != nullptr)
        *error = static_cast<uint8_t>(rxpacket[PKT_ERROR]);
    }
    return result;
}

CommErrorCode Protocol1PacketHandler::ping(PortHandler *port, uint8_t id, uint8_t *error) {
    return ping(port, id, nullptr, error);
}

CommErrorCode Protocol1PacketHandler::ping(PortHandler *port, uint8_t id, uint16_t *model_number, uint8_t *error) {
    uint8_t txpacket[6] = {0};
    uint8_t rxpacket[6] = {0};

    if (id >= _BroadCastId)
        return CommNotImplement;

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH]        = 2;
    txpacket[PKT_INSTRUCTION]   = InstPing;

    CommErrorCode result = txRxPacket(port, txpacket, rxpacket, error);
    if (result == CommSuccess && model_number !=nullptr) {
        uint8_t data_read[2] = {0};
        uint16_t len =2;
        result = readTxRx(port, id, 0, &len, data_read);  // Address 0 : Model Number
        if (result == CommSuccess) *model_number = TI_MAKEWORD(data_read[0], data_read[1]);
    }

    return result;
}

CommErrorCode Protocol1PacketHandler::broadcastPing(PortHandler *port, std::vector<uint8_t> &id_list) {
    (void)port;
    (void)id_list;
    return CommNotImplement;
}

CommErrorCode Protocol1PacketHandler::action(PortHandler *port, uint8_t id) {
    uint8_t txpacket[6] = {0};

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH] = 2;
    txpacket[PKT_INSTRUCTION] = InstAction;

    return txRxPacket(port, txpacket, nullptr);
}

CommErrorCode Protocol1PacketHandler::reboot(PortHandler *port, uint8_t id, uint8_t *error) {
    (void)port;
    (void)id;
    (void)error;
    return CommNotImplement;
}

CommErrorCode Protocol1PacketHandler::factoryReset(PortHandler *port, uint8_t id, uint8_t option, uint8_t *error) {
    (void)option;
    uint8_t txpacket[6] = {0};
    uint8_t rxpacket[6] = {0};

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH] = 2;
    txpacket[PKT_INSTRUCTION] = InstFactoryReset;

    return txRxPacket(port, txpacket, rxpacket, error);
}

CommErrorCode Protocol1PacketHandler::homing(PortHandler *port, uint8_t id, uint8_t direction, uint8_t *error) {
    (void)port;
    (void)id;
    (void)direction;
    (void)error;
    return CommNotImplement;
}

CommErrorCode Protocol1PacketHandler::readTx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length) {
    uint8_t txpacket[8] = {0};

    if (id >= _BroadCastId)
      return CommNotImplement;

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH] = 4;
    txpacket[PKT_INSTRUCTION] = InstRead;
    txpacket[PKT_PARAMETER0+0] = static_cast<uint8_t>(address);
    txpacket[PKT_PARAMETER0+1] = static_cast<uint8_t>(length);

    CommErrorCode result = txPacket(port, txpacket);

    if (result == CommSuccess) // set packet timeout
      port->setPacketTimeout(static_cast<uint16_t>(length+6));

    return result;
}

CommErrorCode Protocol1PacketHandler::readRx(PortHandler *port, uint16_t length, uint8_t *data, uint8_t *error) {
    uint8_t *rxpacket = reinterpret_cast<uint8_t *>(malloc(RXPACKET_MAX_LEN));//(length+6);
    CommErrorCode result = rxPacket(port, rxpacket);
    if (result == CommSuccess) {
        if (error !=nullptr) {
            *error = static_cast<uint8_t>(rxpacket[PKT_ERROR]);
        }
        for (uint8_t s = 0; s < length; s++) {
            data[s] = rxpacket[PKT_PARAMETER0 + s];
        }
    }

    free(rxpacket);
    return result;
}

CommErrorCode Protocol1PacketHandler::readTxRx(PortHandler *port, uint8_t id, uint16_t address,
                                               uint16_t *length, uint8_t *data, uint8_t *error) {
    uint8_t txpacket[8] = {0};
    uint8_t *rxpacket = reinterpret_cast<uint8_t *>(malloc(RXPACKET_MAX_LEN));//(length+6);

    if (id >= _BroadCastId)
        return CommNotImplement;

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH] = 4;
    txpacket[PKT_INSTRUCTION] = InstRead;
    txpacket[PKT_PARAMETER0+0] = static_cast<uint8_t>(address);
    txpacket[PKT_PARAMETER0+1] = static_cast<uint8_t>(*length);

    CommErrorCode result = txRxPacket(port, txpacket, rxpacket, error);
    if (result == CommSuccess) {
        if (error != nullptr) {
            *error = (uint8_t)rxpacket[PKT_ERROR];
        }
        for (uint8_t s = 0; s < *length; s++) {
            data[s] = rxpacket[PKT_PARAMETER0 + s];
        }
    }

    free(rxpacket);
    return result;
}

CommErrorCode Protocol1PacketHandler::writeTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data) {
    uint8_t *txpacket = reinterpret_cast<uint8_t *>(malloc(length+7));

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH] = length+3;
    txpacket[PKT_INSTRUCTION] = InstWrite;
    txpacket[PKT_PARAMETER0] = static_cast<uint8_t>(address);

    for (uint8_t s = 0; s < length; s++)
        txpacket[PKT_PARAMETER0+1+s] = data[s];

    CommErrorCode result = txPacket(port, txpacket);
    port->setUsing(false);

    free(txpacket);
    return result;
}

CommErrorCode Protocol1PacketHandler::writeTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error) {
    uint8_t *txpacket = reinterpret_cast<uint8_t *>(malloc(length+7)); //#6->7
    uint8_t rxpacket[6] = {0};

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH] = length+3;
    txpacket[PKT_INSTRUCTION] = InstWrite;
    txpacket[PKT_PARAMETER0] = static_cast<uint8_t>(address);

    for (uint8_t s = 0; s < length; s++)
        txpacket[PKT_PARAMETER0+1+s] = data[s];

    CommErrorCode result = txRxPacket(port, txpacket, rxpacket, error);
    free(txpacket);
    return result;
}

CommErrorCode Protocol1PacketHandler::regWriteTxOnly(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data) {
    uint8_t *txpacket = reinterpret_cast<uint8_t *>(malloc(length+6));

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH] = static_cast<uint8_t>(length+3);
    txpacket[PKT_INSTRUCTION] = InstRegWrite;
    txpacket[PKT_PARAMETER0] = static_cast<uint8_t>(address);

    for (uint8_t s = 0; s < length; s++)
        txpacket[PKT_PARAMETER0+1+s] = data[s];

    CommErrorCode result = txPacket(port, txpacket);
    port->setUsing(false);

    free(txpacket);
    return result;
}

CommErrorCode Protocol1PacketHandler::regWriteTxRx(PortHandler *port, uint8_t id, uint16_t address, uint16_t length, uint8_t *data, uint8_t *error) {
    uint8_t *txpacket           = reinterpret_cast<uint8_t *>(malloc(length+6));
    uint8_t rxpacket[6]         = {0};

    txpacket[PKT_ID]            = id;
    txpacket[PKT_LENGTH]        = static_cast<uint8_t>(length+3);
    txpacket[PKT_INSTRUCTION]   = InstRegWrite;
    txpacket[PKT_PARAMETER0]    = static_cast<uint8_t>(address);

    for (uint8_t s = 0; s < length; s++)
        txpacket[PKT_PARAMETER0+1+s] = data[s];

    CommErrorCode result = txRxPacket(port, txpacket, rxpacket, error);
    free(txpacket);
    return result;
}

CommErrorCode Protocol1PacketHandler::syncReadTx(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length) {
    (void)port;
    (void)start_address;
    (void)data_length;
    (void)param;
    (void)param_length;
    return CommNotImplement;
}

CommErrorCode Protocol1PacketHandler::syncWriteTxOnly(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, size_t param_length) {
    uint8_t *txpacket           = reinterpret_cast<uint8_t *>(malloc(param_length+8));
    // 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN ... CHKSUM

    txpacket[PKT_ID]            = _BroadCastId;
    txpacket[PKT_LENGTH]        = static_cast<uint8_t>(param_length + 4); // 4: INST START_ADDR DATA_LEN ... CHKSUM
    txpacket[PKT_INSTRUCTION]   = InstSyncWrite;
    txpacket[PKT_PARAMETER0+0]  = static_cast<uint8_t>(start_address);
    txpacket[PKT_PARAMETER0+1]  = static_cast<uint8_t>(data_length);

    for (uint8_t s = 0; s < param_length; s++)
        txpacket[PKT_PARAMETER0+2+s] = param[s];

    CommErrorCode result = txRxPacket(port, txpacket, nullptr, nullptr);
    free(txpacket);
    return result;
}

CommErrorCode Protocol1PacketHandler::bulkReadTx(PortHandler *port, uint8_t *param, size_t param_length) {
    uint8_t *txpacket           = reinterpret_cast<uint8_t *>(malloc(param_length+7));
    // 7: HEADER0 HEADER1 ID LEN INST 0x00 ... CHKSUM

    txpacket[PKT_ID]            = _BroadCastId;
    txpacket[PKT_LENGTH]        = static_cast<uint8_t>(param_length + 3); // 3: INST 0x00 ... CHKSUM
    txpacket[PKT_INSTRUCTION]   = InstBulkRead;
    txpacket[PKT_PARAMETER0+0]  = 0x00;

    for (uint8_t s = 0; s < param_length; s++)
        txpacket[PKT_PARAMETER0+1+s] = param[s];

    CommErrorCode result = txPacket(port, txpacket);
    if (result == CommSuccess) {
        int wait_length = 0;
        for (size_t i = 0; i < param_length; i += 3)
            wait_length += param[i] + 7;
        port->setPacketTimeout(static_cast<uint16_t>(wait_length));
    }

    free(txpacket);
    return result;
}

CommErrorCode Protocol1PacketHandler::bulkWriteTxOnly(PortHandler *port, uint8_t *param, uint16_t param_length) {
    (void)port;
    (void)param;
    (void)param_length;
    return CommNotImplement;
}
