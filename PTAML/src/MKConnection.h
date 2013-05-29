#ifndef __MKCONNECTION_H
#define __MKCONNECTION_H

#include "MKProtocol.h"

#include <cstdarg>
#include <functional>

#include <TooN/TooN.h>

namespace PTAMM {

struct PTAMToMK_t {
  float roll_cmd;  // rad/s^2
  float pitch_cmd;  // rad/s^2
  float yaw_cmd;  // rad/s^2
  int16_t transient_thrust;  // Q9 N
  int16_t hover_thrust;  // Q9 N
  float euler_angles[3];  // rad
  uint8_t status;  // 0x1 = Engaged, 0x2 = Takeoff Mode, 0x4 = Tracking
} __attribute__((packed));

struct MKToPTAM_t {
  float single[3];
  int16_t int16[6];
  uint8_t request;  // 0x1 = Engaged, 0x2 = Takeoff Mode
  uint8_t altitude;  // Q6 m [0, 4]
  uint8_t count;
} __attribute__((packed));

struct MKData_t {
  int16_t int16[9];
} __attribute__((packed));

struct MKDebug_t
{
  uint8_t status[2];
  int16_t int16[32];
} __attribute__((packed));

class MKConnection {
  public:
    typedef std::function<void()> PositionHoldCallback;
    typedef std::function<void(const MKToPTAM_t&)> MKToPTAMCallback;
    typedef std::function<void(const MKData_t&)> MKDataCallback;
    typedef std::function<void(const MKDebug_t&)> MKDebugCallback;

    MKConnection() : mOpen(false) {}
    MKConnection(int comPortId, int baudrate);
    ~MKConnection();

    operator bool() const { return mOpen; }

    void ProcessIncoming();

    void SendPTAMToMK(const double *control, const TooN::Vector<3> &eulerAngles,
        uint8_t status);
    void SendNewTargetNotice();

    void RequestMKDebugInterval(uint8_t interval);

    void SetMKToPTAMCallback(const MKToPTAMCallback &callback) {
      mMKToPTAMCallback = callback;
    }
    void SetMKDataCallback(const MKDataCallback &callback) {
      mMKDataCallback = callback;
    }
    void SetMKDebugCallback(const MKDebugCallback &callback) {
      mMKDebugCallback = callback;
    }
    void SetPositionHoldCallback(const PositionHoldCallback &callback) {
      mPositionHoldCallback = callback;
    }

  private:
    void HandleMKToPTAM(const SerialMsg_t& msg);
    void HandleMKData(const SerialMsg_t& msg);
    void HandleMKDebug(const SerialMsg_t& msg);

    void SendData(uint8_t cmdID, uint8_t dataLength, ...);

    bool mOpen;
    int mComPortId;

    enum { TX_BUFFER_SIZE = 4096 };
    uint8_t mTxBufferData[TX_BUFFER_SIZE];
    enum { RX_BUFFER_SIZE = 4096 };
    uint8_t mRxBufferData[RX_BUFFER_SIZE];

    Buffer_t mRxBuffer;

    // Callbacks
    MKToPTAMCallback mMKToPTAMCallback;
    MKDataCallback mMKDataCallback;
    MKDebugCallback mMKDebugCallback;
    PositionHoldCallback mPositionHoldCallback;
};

}

#endif
