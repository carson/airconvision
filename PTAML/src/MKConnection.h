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


struct CtrlRqst_t
{
    uint8_t ConfigRqst;
    int16_t HoverGas;
} __attribute__((packed));


struct ExternControl_t {
    int8_t Pitch;
    int8_t Roll;
    int8_t Yaw;
    int8_t Gas;
    uint8_t HoverGas;     // "Gas" (throttle) setting required for hover
    uint8_t Config;       // 0x1 - Engaged, 0x2 - Takeoff, 0x4 - Tracking
} __attribute__((packed));


class MKConnection {
  public:
    typedef std::function<void()> PositionHoldCallback;
    typedef std::function<void(const CtrlRqst_t&)> ControlRqstCallback;
    typedef std::function<void(const DebugOut_t&)> DebugOutputCallback;

    MKConnection() : mOpen(false) {}
    MKConnection(int comPortId, int baudrate);
    ~MKConnection();

    operator bool() const { return mOpen; }

    void ProcessIncoming();

    void SendNewTargetNotice();

    void SendExternControl(const double* control, const uint8_t config);

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
