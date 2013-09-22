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
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;
    typedef std::chrono::duration<double> RealSeconds;

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
    const double(& GetControl() const)[5] { return mTargetController.GetControl(); }
    double GetTargetAltitude() { return mTargetController.GetTargetAltitude(); }
    uint8_t GetConfig() const { return mTargetController.GetConfig(); }
    const float* GetMKData() const { return mMKToPTAM.single; }

  private:
    void ConnectToFC(int nComPortId, int nComBaudrate);
    void ConnectToNavi(int nComPortId, int nComBaudrate);

    // MK message handlers
    void RecvPositionHold();
    void RecvMKToPTAM(const MKToPTAM_t& MKToPTAM);
    void RecvMKDebug(const MKDebug_t& mkDebug);
    void RecvMKNavi(const MKNavi_t& mkNavi);

    void LogMKControl();
    void LogMKData();
    void LogMKDebug();
    void LogMKNavi();

    mutable std::mutex mMutex;

    bool mbDone;
    const Tracker* mpTracker;
    PerformanceMonitor *mpPerfMon;
    bool mbHasTracking;
    bool mbUpdateReady;
    bool mbLogMKControl;
    bool mbLogMKData;
    bool mbLogMKDebug;
    bool mbLogMKNavi;
    TimeoutTimer mMKDebugRequestTimeout;
    TimeoutTimer mMKNaviRequestTimeout;
    MKData_t mMKData;
    MKDebug_t mMKDebug;
    MKNavi_t mMKNavi;
    MKToPTAM_t mMKToPTAM;

    MKConnection mFCConn;
    MKConnection mNaviConn;
    TargetController mTargetController;

    std::ofstream mMKControlLogFile;
    std::ofstream mMKDataLogFile;
    std::ofstream mMKDebugLogFile;
    std::ofstream mMKNaviLogFile;

    TimePoint mStartTime;
};

}

#endif
