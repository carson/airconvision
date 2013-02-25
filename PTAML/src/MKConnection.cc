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
  RXCMD_CONTROL_RQST = 'E',
  RXCMD_CONTROL_CNFRM = 'B',
  RXCMD_POSITION_HOLD = 'H',
  RXCMD_DEBUG_OUTPUT = 'D'
};

enum TxCommands : uint8_t {
  TXCMD_NEW_TARGET = 'x',
  TXCMD_DEBUG_OUTPUT = 'd',
  TXCMD_EXTERN_CONTROL = 'b'
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

int32_t DistanceToInt32(double dist)
{
  return int32_t(dist * 1000.0 + 0.5);
}

void MKConnection::SendNewTargetNotice()
{
  assert(mOpen);

  // Initialize a buffer for the packet
  Buffer_t txBuffer;
  Buffer_Init(&txBuffer, mTxBufferData, TX_BUFFER_SIZE);

  MKProtocol_CreateSerialFrame(&txBuffer, TXCMD_NEW_TARGET, FC_ADDRESS, 0);

  // Send txBuffer to NaviCtrl
  SendBuffer(txBuffer);
}

void MKConnection::SendExternControl(const double *control, const uint8_t config)
{
  assert(mOpen);

  // Initialize a buffer for the packet
  Buffer_t txBuffer;
  Buffer_Init(&txBuffer, mTxBufferData, TX_BUFFER_SIZE);

  ExternControl_t data;
  data.Pitch = int16_t(control[1] * 256. / M_PI + 0.5);
  data.Roll = int16_t(control[0] * 256. / M_PI + 0.5);
  data.Yaw = int8_t(control[2] * 256. / M_PI + 0.5);
  data.Gas = int8_t(control[3] * 4 + 0.5);
  data.HoverGas = uint8_t(control[4] * 4 + 0.5);
  data.Config = config;
  // TODO: Delivery confirmation

  MKProtocol_CreateSerialFrame(&txBuffer, TXCMD_EXTERN_CONTROL, FC_ADDRESS, 1, (uint8_t*)&data, sizeof(ExternControl_t));

  // Send txBuffer to NaviCtrl
  SendBuffer(txBuffer);
}

void MKConnection::SendDebugOutputInterval(uint8_t interval)
{
  assert(mOpen);

  // Initialize a buffer for the packet
  Buffer_t txBuffer;
  Buffer_Init(&txBuffer, mTxBufferData, TX_BUFFER_SIZE);

  MKProtocol_CreateSerialFrame(&txBuffer, TXCMD_DEBUG_OUTPUT, FC_ADDRESS, 1, (uint8_t*)&interval, 1);
  //cout << "Debug Request Sent" << endl;

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

        // Reset the buffer so it can be used for next message
        Buffer_Clear(&mRxBuffer);

        if (msg.Address == FC_ADDRESS) {
          // Messages addressed from FlightCtrl
          switch (msg.CmdID) {
          case RXCMD_CONTROL_RQST:
            cout << "Received control request" << endl;
            HandleControlRqst(msg);
            break;
          case RXCMD_CONTROL_CNFRM:
            // cout << "Received control receipt confirmation" << endl;
            break;
          case RXCMD_POSITION_HOLD:
            cout << "Received position hold request" << endl;
            mPositionHoldCallback();
            break;
          case RXCMD_DEBUG_OUTPUT:
            // cout << "Received debug data" << endl;
            HandleDebugOutput(msg);
            break;
          default:
            cerr << "Unknown MK FlightCtrl command received: " << msg.CmdID << endl;
            break;
          }
//      } else {
//        cerr << "Unknown MK command \'" << msg.CmdID << "\' addresses to: #" << (int)msg.Address << endl;
        }
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

void MKConnection::HandleControlRqst(const SerialMsg_t& msg)
{
  const CtrlRqst_t *pControlRqst = reinterpret_cast<const CtrlRqst_t*>(msg.pData);
  mControlRqstCallback(*pControlRqst);
}

}
