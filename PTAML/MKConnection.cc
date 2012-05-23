#include "MKConnection.h"

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cassert>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232.h"

using namespace std;

namespace PTAMM {

enum RxCommand : uint8_t {
  RXCMD_POSITION_HOLD = 'H',
  RXCMD_DEBUG_OUTPUT
};

enum TxCommands : uint8_t {
  TXCMD_POSITION_DELTA = 'p',
  TXCMD_DEBUG_OUTPUT = 'd'
};

MKConnection::MKConnection(int comPortId, int baudrate)
  : mOpen(false)
  , mComPortId(comPortId)
{
  if (OpenComport(comPortId, baudrate) == 0) {
    mOpen = true;
  }

  if (!mOpen) {
    cerr << "Failed to open COM port #" << comPortId << " @ " << baudrate << endl;
  }

  Buffer_Init(&mRxBuffer, mRxBufferData, RX_BUFFER_SIZE);
}

MKConnection::~MKConnection() {
}

void MKConnection::SendPositionHoldUpdate(const TooN::Vector<3>& v3Offset,
                                          const TooN::Vector<3>& v3Vel)
{
  assert(mOpen);

  // Initialize a buffer for the packet
  Buffer_t txBuffer;
  Buffer_Init(&txBuffer, mTxBufferData, TX_BUFFER_SIZE);

  // Build the packet
  uint8_t positionData[24];

  // v3Offset
  *(int32_t*)&positionData[0] = (int32_t)(v3Offset[0] * 1000 + 0.5);
  *(int32_t*)&positionData[4] = (int32_t)(v3Offset[1] * 1000 + 0.5);
  *(int32_t*)&positionData[8] = (int32_t)(v3Offset[2] * 1000 + 0.5);
  // v3Vel
  *(int32_t*)&positionData[12] = (int32_t)(v3Vel[0] * 1000 + 0.5);
  *(int32_t*)&positionData[16] = (int32_t)(v3Vel[1] * 1000 + 0.5);
  *(int32_t*)&positionData[20] = (int32_t)(v3Vel[2] * 1000 + 0.5);

  MKProtocol_CreateSerialFrame(&txBuffer, TXCMD_POSITION_DELTA, NC_ADDRESS, 1, positionData, 24);

  // Send txBuffer to NaviCtrl
  SendBuffer(txBuffer);
}

void MKConnection::SendDebugOutputInterval(uint8_t interval)
{
  assert(mOpen);

  // Initialize a buffer for the packet
  Buffer_t txBuffer;
  Buffer_Init(&txBuffer, mTxBufferData, TX_BUFFER_SIZE);

  // Build the packet
  uint8_t data[1] = { interval };

  MKProtocol_CreateSerialFrame(&txBuffer, TXCMD_DEBUG_OUTPUT, NC_ADDRESS, 1, data, 1);

  // Send txBuffer to NaviCtrl
  SendBuffer(txBuffer);
}

void MKConnection::ProcessIncoming()
{
  assert(mOpen);

  const int RX_BUFFER_SIZE = 4096;
  uint8_t buffer[RX_BUFFER_SIZE];
  int readBytes = 0;

  do {
    // Read from the COM port
    readBytes = PollComport(mComPortId, buffer, RX_BUFFER_SIZE);

    // Handle the incoming data
    for (int i = 0; i < readBytes; ++i) {
      if (MKProtocol_CollectSerialFrame(&mRxBuffer, buffer[i])) {
        SerialMsg_t msg;
        MKProtocol_DecodeSerialFrameHeader(&mRxBuffer, &msg);
        MKProtocol_DecodeSerialFrameData(&mRxBuffer, &msg);

        // Ignore messages not sent to the OnBoardComputer
        if (msg.Address == OBC_ADDRESS) {
          switch (msg.CmdID) {
          case RXCMD_POSITION_HOLD:
            mPositionHoldCallback();
            break;
          case RXCMD_DEBUG_OUTPUT:
            HandleDebugOutput(msg);
            break;
          default:
            cerr << "Unknown MikroKopter command received: " << msg.CmdID << endl;
            break;
          }
        }

        // Reset the buffer so it can be used for next message
        Buffer_Clear(&mRxBuffer);
      }
    }

  } while (readBytes == RX_BUFFER_SIZE);
}

void MKConnection::SendBuffer(const Buffer_t& txBuffer)
{
  uint16_t length = txBuffer.DataBytes;
  uint8_t* data = txBuffer.pData;

  while (length > 0) {
    int sent = SendBuf(mComPortId, data, length);
    length -= sent;
    data += sent;
  }
}

void MKConnection::HandleDebugOutput(const SerialMsg_t& msg)
{
  const DebugOut_t *pDebugData = reinterpret_cast<const DebugOut_t*>(msg.pData);
  mDebugOutputCallback(*pDebugData);
}

}
