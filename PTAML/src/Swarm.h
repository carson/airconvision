#ifndef __SWARM_H
#define __SWARM_H

#include <TooN/TooN.h>
#include <TooN/se3.h>

#include <thread>
#include <chrono>
#include <mutex>

namespace PTAMM {

class SwarmLab {
  public:
    typedef std::chrono::high_resolution_clock::time_point HiResTimePoint;

    SwarmLab();
    void operator()();
    void StopThread() { mbDone = true; }
    void UpdatePose(const TooN::SE3<> &se3Pose, bool bHasTracking,
                    const HiResTimePoint &tpTime);
  private:
    void SendPosePacket();

    uint16_t Checksum(const uint8_t* data, size_t length) const;
    void SendBuffer(uint8_t* data, int length);
  private:
    bool mbDone;
    std::mutex mMutex;

    int mnComPortId;
    bool mbOpen;

    TooN::SE3<> mse3CurrentPose;
    HiResTimePoint mtpPoseTime;
    bool mbHasTracking;
    bool mbPoseUpdated;
};

}

#endif
