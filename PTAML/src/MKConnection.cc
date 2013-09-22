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
  RXCMD_MK_TO_PTAM    = 'E',
  RXCMD_MK_DEBUG      = 'D',
  RXCMD_MK_DATA       = 'I',
  RXCMD_POSITION_HOLD = 'H',
  RXCMD_MK_NAVI       = 'O',
};

enum TxCommands : uint8_t {
  TXCMD_MK_DEBUG_RQST = 'd',
  TXCMD_PTAM_TO_MK    = 'b',
  TXCMD_MK_DATA_RQST  = 'i',
  TXCMD_MK_NAVI_RQST  = 'o',
  TXCMD_NEW_TARGET    = 'x',
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

        // cout << "Received CmdID: " << msg.CmdID << " at address: "
        //     << (int16_t)msg.Address << endl;

        // if (msg.Address == FC_ADDRESS) {
          // Messages addressed from FlightCtrl
          switch (msg.CmdID) {
          case RXCMD_MK_TO_PTAM:
            HandleMKToPTAM(msg);
            break;
          case RXCMD_MK_DATA:
            HandleMKData(msg);
            break;
          case RXCMD_MK_DEBUG:
            HandleMKDebug(msg);
            break;
          case RXCMD_MK_NAVI:
            HandleMKNavi(msg);
            break;
          case RXCMD_POSITION_HOLD:
            cout << "Received position hold request" << endl;
            mPositionHoldCallback();
            break;
          default:
            cerr << "Unknown MK FlightCtrl command received: " << msg.CmdID << endl;
            break;
          }
//      } else {
//        cerr << "Unknown MK command \'" << msg.CmdID << "\' addresses to: #" << (int)msg.Address << endl;
        // }
      }
    }

  } while (readBytes == RX_BUFFER_SIZE);
}

void MKConnection::SendPTAMToMK(const double* control,
    const TooN::Vector<3>& eulerAngles, uint8_t status)
{
  PTAMToMK_t data;
  data.roll_cmd = control[0];
  data.pitch_cmd = control[1];
  data.yaw_cmd = control[2];
  data.transient_thrust = (int16_t)(control[3] * 512. + 0.5);  // Q9 N [-10, 10]
  data.hover_thrust = (int16_t)(control[4] * 512. + 0.5);  // Q9 N [-30, 15]
  for (uint8_t i = 0; i < 3; i++)
    data.euler_angles[i] = eulerAngles[i];
  data.status = status;

  SendData(TXCMD_PTAM_TO_MK, sizeof(PTAMToMK_t), (uint8_t*)&data);
  // TODO: Delivery confirmation
}

void MKConnection::SendNewTargetNotice()
{
  SendData(TXCMD_NEW_TARGET, 0);
}

void MKConnection::RequestMKDebugInterval(uint8_t interval)
{
  SendData(TXCMD_MK_DEBUG_RQST, 1, &interval);
}

void MKConnection::RequestMKNaviInterval(uint8_t interval)
{
  SendData(TXCMD_MK_NAVI_RQST, 1, &interval);
}

void MKConnection::HandleMKToPTAM(const SerialMsg_t& msg)
{
  const MKToPTAM_t *pMKToPTAM = reinterpret_cast<const MKToPTAM_t*>(msg.pData);
  mMKToPTAMCallback(*pMKToPTAM);
}

void MKConnection::HandleMKData(const SerialMsg_t& msg)
{
  const MKData_t *pMKData = reinterpret_cast<const MKData_t*>(msg.pData);
  mMKDataCallback(*pMKData);
}

void MKConnection::HandleMKDebug(const SerialMsg_t& msg)
{
  const MKDebug_t *pMKDebug = reinterpret_cast<const MKDebug_t*>(msg.pData);
  mMKDebugCallback(*pMKDebug);
}

void MKConnection::HandleMKNavi(const SerialMsg_t& msg)
{
  const MKNavi_t *pMKNavi = reinterpret_cast<const MKNavi_t*>(msg.pData);
  mMKNaviCallback(*pMKNavi);
}

void MKConnection::SendData(uint8_t cmdID, uint8_t dataLength, ...)  //uint8_t *data
{
  va_list ap;
  Buffer_t txBuffer;  // Initialize a buffer for the packet
  uint8_t* pdata = NULL;

  assert(mOpen);
  Buffer_Init(&txBuffer, mTxBufferData, TX_BUFFER_SIZE);  // Initialize buffer

  va_start(ap, dataLength);
  if(dataLength) {
    pdata = va_arg(ap, uint8_t*);
    MKProtocol_CreateSerialFrame(&txBuffer, cmdID, FC_ADDRESS, 1, pdata,
        dataLength);
  } else {
    MKProtocol_CreateSerialFrame(&txBuffer, cmdID, FC_ADDRESS, 0);
  }
  va_end(ap);

  uint16_t length = txBuffer.DataBytes;
  uint8_t* data = txBuffer.pData;
  while (length > 0) {
    int sent = SendBuf(mComPortId, data, length);
    length -= sent;
    data += sent;
  }
}

}
