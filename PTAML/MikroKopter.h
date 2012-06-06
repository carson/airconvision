#ifndef __MK_CONTROLLER_H
#define __MK_CONTROLLER_H

#include "MKConnection.h"
#include "TargetController.h"
#include "Timing.h"

#include <TooN/se3.h>

#include <fstream>
#include <chrono>

namespace PTAMM {

class MikroKopter {
  public:
    MikroKopter();

    void Update(const TooN::SE3<> &se3Pose, bool bHasTracking);

    void GoToPosition(const TooN::SE3<> &se3PoseInWorld);
    void AddWaypoint(const TooN::SE3<> &se3PoseInWorld);
    void ClearWaypoints();
    void FlyPath();

  private:
    void ConnectToMK(int nComPortId, int nComBaudrate);
    void LogControlValues();

    // MK message handlers
    void RecvRequestPositionHold();
    void RecvDebugOutput(const DebugOut_t& debug);

  private:
    enum ControllerType {
      NO_CONTROLLER,
      TARGET_CONTROLLER,
      PATH_CONTROLLER
    };

    MKConnection mMkConn;

    ControllerType mControllerType;
    TargetController mTargetController;

    DebugOut_t mMkDebugOutput;
    TimeoutTimer mSendDebugTimeout;

    bool mbWriteControlValuesLog;
    std::ofstream mControlValuesFile;

};

}

#endif
