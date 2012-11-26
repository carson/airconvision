#include "MikroKopter.h"
#include "PerformanceMonitor.h"
#include "Tracker.h"

#include <gvars3/instances.h>

#include <functional>
#include <fstream>
#include <thread>
#include <chrono>

using namespace std;
using namespace std::placeholders;
using namespace GVars3;

namespace PTAMM {

MikroKopter::MikroKopter(const Tracker* pTracker, PerformanceMonitor *pPerfMon)
  : mbDone(false)
  , mpTracker(pTracker)
  , mpPerfMon(pPerfMon)
  , mControllerType(TARGET_CONTROLLER)
  , mbHasTracking(false)
  , mSendDebugTimeout(2.0)
  , mbWriteControlValuesLog(false)
{
  // Set all debug output values to 0
  mMkDebugOutput = { 0 };

  // Debug output of all the control variables and debug probes sent from the MK
  mbWriteControlValuesLog = GV3::get<int>("Debug.OutputControlValues", 0, SILENT);
  if (mbWriteControlValuesLog) {
    mControlValuesFile.open("mk_control.txt", ios::out | ios::trunc);
    if (!mControlValuesFile) {
      cerr << "Failed to open mk_control.txt" << endl;
    }
  }

  // Read COM port settings
  int nComPortId = GV3::get<int>("MKNaviCtrl.ComPortId", 16, SILENT); // 16 is /dev/ttyUSB0
  int nCcomPortBaudrate = GV3::get<int>("MKNaviCtrl.ComPortBaudrate", "57600", SILENT);
  ConnectToMK(nComPortId, nCcomPortBaudrate);
}

void MikroKopter::operator()()
{
  RateLimiter rateLimiter;
  static int counter = 0;

  while (!mbDone) {
    if (mMkConn) {
      //const TooN::Vector<3>& v3Offset = mTargetController.GetTargetOffsetFiltered();
      //cout << v3Offset[2] << endl;
      mMkConn.ProcessIncoming();

      // Don't try to send two commands on the same frame
      if (counter % 2) { // 10Hz
        std::unique_lock<std::mutex> lock(mMutex);

        mMkConn.SendExternControl(mTargetController.GetControl(), mTargetController.GetConfig());

        // Check if the debug values from the MK and position hold code should be written to a log
        // TODO: move this to a callback or something to avoid weird timing issues
        if (mbWriteControlValuesLog && (counter % 4)) { // 5Hz (to match the request below)
          LogControlValues();
        }
      }
      // Request debug data being sent from the MK, this has to be done every few seconds or
      // the MK will stop sending the data
      else if (mSendDebugTimeout.HasTimedOut()) {
        mMkConn.SendDebugOutputInterval(20);  // Request data at 5 Hz
        mSendDebugTimeout.Reset();
      }
    }

    // Lock rate to 20 Hz
    rateLimiter.Limit(20.0);

    mpPerfMon->UpdateRateCounter("mk");
    counter++;
  }
}

void MikroKopter::UpdatePose(const TooN::SE3<> &se3Pose, bool bHasTracking)
{
  std::unique_lock<std::mutex> lock(mMutex);
  mbHasTracking = bHasTracking;
  mTargetController.Update(se3Pose, bHasTracking, TargetController::Clock::now());
}

void MikroKopter::GoToPosition(const TooN::SE3<> &se3PoseInWorld)
{
  std::unique_lock<std::mutex> lock(mMutex);
  cout << "Go to position: " << se3PoseInWorld.get_translation() << endl;
  mTargetController.SetTarget(se3PoseInWorld);
  mMkConn.SendNewTargetNotice();
  mControllerType = TARGET_CONTROLLER;
}

void MikroKopter::AddWaypoint(const TooN::SE3<> &se3PoseInWorld)
{
  std::unique_lock<std::mutex> lock(mMutex);
  cout << "Adding waypoint: " << se3PoseInWorld.get_translation() << endl;
  mPathController.AddWaypoint(se3PoseInWorld);
}

void MikroKopter::ClearWaypoints()
{
  std::unique_lock<std::mutex> lock(mMutex);
  mPathController.ClearWaypoints();
}

void MikroKopter::FlyPath()
{
  std::unique_lock<std::mutex> lock(mMutex);
  mPathController.Start();
  mControllerType = NO_CONTROLLER;
}


void MikroKopter::ConnectToMK(int nComPortId, int nComBaudrate)
{
  // Attempt connecting to the MK NaviCtrl
  mMkConn = MKConnection(nComPortId, nComBaudrate);
  if (!mMkConn) {
    cerr << "Failed to connect to MikroKopter NaviCtrl." << endl;
  } else {
    // Hook up all the callback functions
    mMkConn.SetPositionHoldCallback(std::bind(&MikroKopter::RecvPositionHold, this));
    mMkConn.SetControlRqstCallback(std::bind(&MikroKopter::RecvControlRqst, this, _1));
    mMkConn.SetDebugOutputCallback(std::bind(&MikroKopter::RecvDebugOutput, this, _1));
    // Request debug data being sent from the MK to this computer
    mMkConn.SendDebugOutputInterval(50);
  }
}

void MikroKopter::LogControlValues()
{
  mControlValuesFile
    << mTargetController.GetTime() << " "
//    << mTargetController.GetTargetOffset() << " "
    << mTargetController.GetTargetOffsetFiltered() << " "
//    << mTargetController.GetVelocity() << " "
    << mTargetController.GetVelocityFiltered();

  for (size_t i = 0; i < 32; ++i) {
    mControlValuesFile << " " << mMkDebugOutput.Analog[i];
  }

  mControlValuesFile << endl;
}

void MikroKopter::RecvPositionHold()
{
  cout << " >> Position hold requested." << endl;
  GoToPosition(mpTracker->GetCurrentPose().inverse());
}

void MikroKopter::RecvControlRqst(const CtrlRqst_t& control)
{
  cout << " >> Config " << (int)control.ConfigRqst << " requested from config "
      << (int)(mTargetController.GetConfig() & 0x3) << endl;
  mTargetController.RequestConfig(control.ConfigRqst, control.HoverGas);
}

void MikroKopter::RecvDebugOutput(const DebugOut_t& debugData)
{
  mMkDebugOutput = debugData;
}

}
