#ifndef __MK_CONTROLLER_H
#define __MK_CONTROLLER_H

#include "MKConnection.h"

#include <chrono>
#include <fstream>
#include <mutex>

#include <TooN/se3.h>

#include "TargetController.h"
#include "Timing.h"

namespace PTAMM {

class Tracker;
class PerformanceMonitor;

class MikroKopter {
  public:
    MikroKopter(const Tracker* pTracker, PerformanceMonitor *pPerfMon);

    void operator()();

    void StopThread() { mbDone = true; }

    void UpdatePose(const TooN::SE3<> &se3Pose, bool bHasTracking);

    void GoToLocation(TooN::Vector<2> v2LocInWorld);
    void SetTargetAltitude(double altitude);

    // To access private data
    const TooN::Vector<3>& GetPosInWorld() const { return mTargetController.GetPosInWorld(); }
    const TooN::Vector<3>& GetEulerAngles() const { return mTargetController.GetEulerAngles(); }
    const TooN::Vector<3>& GetTargetOffset() const { return mTargetController.GetTargetOffset(); }
    const TooN::Vector<3>& GetVelocity() const { return mTargetController.GetVelocity(); }
    const double* GetControl() const { return mTargetController.GetControl(); }
    double GetTargetAltitude() { return mTargetController.GetTargetAltitude(); }
    uint8_t GetConfig() const { return mTargetController.GetConfig(); }
    const float* GetMKData() const { return mMKToPTAM.single; }

  private:
    void ConnectToMK(int nComPortId, int nComBaudrate);

    // MK message handlers
    void RecvPositionHold();
    void RecvMKToPTAM(const MKToPTAM_t& MKToPTAM);
    void RecvMKDebug(const MKDebug_t& mkDebug);

    void LogMKData();
    void LogMKDebug();

    mutable std::mutex mMutex;

    bool mbDone;
    const Tracker* mpTracker;
    PerformanceMonitor *mpPerfMon;
    bool mbHasTracking;
    bool mbUpdateReady;
    bool mbLogMKData;
    bool mbLogMKDebug;
    TimeoutTimer mMKDebugRequestTimeout;
    MKData_t mMKData;
    MKDebug_t mMKDebug;
    MKToPTAM_t mMKToPTAM;

    MKConnection mMkConn;
    TargetController mTargetController;

    std::ofstream mMKDataLogFile;
    std::ofstream mMKDebugLogFile;
};

}

#endif
