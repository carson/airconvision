#ifndef __MKCONNECTION_H
#define __MKCONNECTION_H

#include <TooN/TooN.h>
#include "MKProtocol.h"

namespace PTAMM {

class MKConnection {
  public:
    MKConnection(int comPortId, int baudrate);
    ~MKConnection();

    operator bool() const { return mOpen; }

    void SendPosition(const TooN::Vector<3>& v3Position);

    void ProcessIncoming();

  private:
    void SendBuffer(const Buffer_t& txBuffer);

    bool mOpen;
    int mComPortId;

    enum { TX_BUFFER_SIZE = 4096 };
    uint8_t mTxBufferData[TX_BUFFER_SIZE];
};

}

#endif
