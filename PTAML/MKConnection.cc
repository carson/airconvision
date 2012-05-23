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

enum Commands : uint8_t {
  CMD_POSITION_DELTA = 'p'
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

  // @TODO Encode position into positionData. Remember byte order.
  MKProtocol_CreateSerialFrame(&txBuffer, CMD_POSITION_DELTA, NC_ADDRESS, 1, positionData, 24);

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

        // Ignore messages not sent to the OnBoardComputer
        if (msg.Address == OBC_ADDRESS) {
          switch (msg.CmdID) {
          case 'H': // Position hold
            mPositionHoldCallback();
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

}
