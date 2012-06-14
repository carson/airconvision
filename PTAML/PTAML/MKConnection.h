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
    uint16_t Analog[32];
} __attribute__((packed));


class MKConnection {
  public:
    typedef std::function<void()> PositionHoldCallback;
    typedef std::function<void(const DebugOut_t&)> DebugOutputCallback;

    MKConnection() : mOpen(false) {}
    MKConnection(int comPortId, int baudrate);
    ~MKConnection();

    operator bool() const { return mOpen; }

    void ProcessIncoming();

    void SendPositionHoldUpdate(const TooN::Vector<3> &v3OffsetToTargetInCam,
                                const TooN::Vector<3> &v3VelocityInCam);

    void SendDebugOutputInterval(uint8_t interval);

    void SetPositionHoldCallback(const PositionHoldCallback &callback) {
      mPositionHoldCallback = callback;
    }

    void SetDebugOutputCallback(const DebugOutputCallback &callback) {
      mDebugOutputCallback = callback;
    }

  private:
    void SendBuffer(const Buffer_t& txBuffer);
    void HandleDebugOutput(const SerialMsg_t& msg);

    bool mOpen;
    int mComPortId;

    enum { TX_BUFFER_SIZE = 4096 };
    uint8_t mTxBufferData[TX_BUFFER_SIZE];
    enum { RX_BUFFER_SIZE = 4096 };
    uint8_t mRxBufferData[RX_BUFFER_SIZE];

    Buffer_t mRxBuffer;

    // Callbacks
    PositionHoldCallback mPositionHoldCallback;
    DebugOutputCallback mDebugOutputCallback;
};

}

#endif
