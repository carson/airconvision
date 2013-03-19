#ifndef __MKCONNECTION_H
#define __MKCONNECTION_H

#include "MKProtocol.h"
#include <TooN/TooN.h>
#include <functional>

namespace PTAMM {

struct DebugOut_t
{
    uint8_t StatusGreen;
    uint8_t StatusRed;
    int16_t Analog[32];
} __attribute__((packed));


struct ExternControl_t {
    int16_t roll;   // Q9 PI*rad/s^2
    int16_t pitch;  // Q9 PI*rad/s^2
    int16_t yaw;    // Q9 PI*rad/s^2
    int16_t transient_thrust;  // Q9 N
    int16_t hover_thrust;  // Q9 N
    int16_t euler_angles[3];  // Q12 PI*rad
    uint8_t status;  // 0x1 = Engaged, 0x2 = Takeoff Mode, 0x4 = Tracking
} __attribute__((packed));


struct ControlRequest_t {
    uint8_t status;  // 0x1 = Engaged, 0x2 = Takeoff Mode
    uint8_t altitude;  // Q6 m [0, 4]
} __attribute__((packed));


class MKConnection {
  public:
    typedef std::function<void()> PositionHoldCallback;
    typedef std::function<void(const ControlRequest_t&)> ControlRqstCallback;
    typedef std::function<void(const DebugOut_t&)> DebugOutputCallback;

    MKConnection() : mOpen(false) {}
    MKConnection(int comPortId, int baudrate);
    ~MKConnection();

    operator bool() const { return mOpen; }

    void ProcessIncoming();

    void SendNewTargetNotice();

    void SendExternControl(const double *control,
        const TooN::Vector<3> &eulerAngles, uint8_t status);

    void SendDebugOutputInterval(uint8_t interval);

    void SetPositionHoldCallback(const PositionHoldCallback &callback) {
      mPositionHoldCallback = callback;
    }

    void SetControlRqstCallback(const ControlRqstCallback &callback) {
      mControlRqstCallback = callback;
    }

    void SetDebugOutputCallback(const DebugOutputCallback &callback) {
      mDebugOutputCallback = callback;
    }

  private:
    void SendBuffer(const Buffer_t& txBuffer);
    void HandleDebugOutput(const SerialMsg_t& msg);
    void HandleControlRqst(const SerialMsg_t& msg);

    bool mOpen;
    int mComPortId;

    enum { TX_BUFFER_SIZE = 4096 };
    uint8_t mTxBufferData[TX_BUFFER_SIZE];
    enum { RX_BUFFER_SIZE = 4096 };
    uint8_t mRxBufferData[RX_BUFFER_SIZE];

    Buffer_t mRxBuffer;

    // Callbacks
    PositionHoldCallback mPositionHoldCallback;
    ControlRqstCallback mControlRqstCallback;
    DebugOutputCallback mDebugOutputCallback;
};

}

#endif
