#ifndef __MKCONNECTION_H
#define __MKCONNECTION_H

#include "MKProtocol.h"
#include <TooN/TooN.h>
#include <functional>

namespace PTAMM {

class MKConnection {
  public:
    MKConnection() : mOpen(false) {}
    MKConnection(int comPortId, int baudrate);
    ~MKConnection();

    operator bool() const { return mOpen; }

    void ProcessIncoming();

    void SendPosition(const TooN::Vector<3>& v3Position);

    void SetPositionHoldCallback(const std::function<void()>& callback) {
      mPositionHoldCallback = callback;
    }

  private:
    void SendBuffer(const Buffer_t& txBuffer);

    bool mOpen;
    int mComPortId;

    enum { TX_BUFFER_SIZE = 4096 };
    uint8_t mTxBufferData[TX_BUFFER_SIZE];
    enum { RX_BUFFER_SIZE = 4096 };
    uint8_t mRxBufferData[RX_BUFFER_SIZE];

    Buffer_t mRxBuffer;

    // Callbacks
    std::function<void()> mPositionHoldCallback;
};

}

#endif
