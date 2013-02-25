#ifndef __MK_CONTROLLER_H
#define __MK_CONTROLLER_H

#include "MKConnection.h"
#include "TargetController.h"
#include "PathController.h"
#include "Timing.h"

#include <TooN/se3.h>

#include <fstream>
#include <chrono>
#include <mutex>

namespace PTAMM {

class Tracker;
class PerformanceMonitor;

class MikroKopter {
  public:
    MikroKopter(const Tracker* pTracker, PerformanceMonitor *pPerfMon);

    void operator()();

    void StopThread() { mbDone = true; }

    void UpdatePose(const TooN::SE3<> &se3Pose, bool bHasTracking);

    void GoToPosition(const TooN::SE3<> &se3PoseInWorld);
    void AddWaypoint(const TooN::SE3<> &se3PoseInWorld);
    void ClearWaypoints();
    void FlyPath();

    const TooN::Vector<3>& GetPosInWorld() const { return mTargetController.GetPosInWorld(); }
    const TooN::Vector<3>& GetEulerAngles() const { return mTargetController.GetEulerAngles(); }

    const TooN::Vector<3>& GetTargetOffset() const { return mTargetController.GetTargetOffset(); }
    const TooN::Vector<3>& GetVelocity() const { return mTargetController.GetVelocity(); }

    const double* GetControl() const { return mTargetController.GetControl(); }
    uint8_t GetConfig() const { return mTargetController.GetConfig(); }

  private:
    void ConnectToMK(int nComPortId, int nComBaudrate);
    void LogControlValues();

    // MK message handlers
    void RecvPositionHold();
    void RecvControlRqst(const CtrlRqst_t& control);
    void RecvDebugOutput(const DebugOut_t& debug);

  private:
    enum ControllerType {
      NO_CONTROLLER,
      TARGET_CONTROLLER,
      PATH_CONTROLLER
    };

    bool mbDone;

    const Tracker* mpTracker;
    PerformanceMonitor *mpPerfMon;

    std::mutex mMutex;

    MKConnection mMkConn;

    ControllerType mControllerType;
    TargetController mTargetController;
    PathController mPathController;

    bool mbHasTracking;

    DebugOut_t mMkDebugOutput;
    TimeoutTimer mSendDebugTimeout;

    bool mbWriteControlValuesLog;
    std::ofstream mControlValuesFile;
};

}

#endif
