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
      mMkConn.ProcessIncoming();

      // Don't try to send two commands on the same frame
      if (mSendDebugTimeout.HasTimedOut()) {
        // Request debug data being sent from the MK, this has to be done every few seconds or
        // the MK will stop sending the data
        mMkConn.SendDebugOutputInterval(10);  // Request data every 100ms (10 Hz)
        mSendDebugTimeout.Reset();
      }
      else {
        std::unique_lock<std::mutex> lock(mMutex);

        mMkConn.SendExternControl(mTargetController.GetControl(),
        mTargetController.GetEulerAngles(), mTargetController.GetConfig());

        // Check if the debug values from the MK and position hold code should be written to a log
        // TODO: move this to a callback or something to avoid weird timing issues
        if (mbWriteControlValuesLog && !(counter % 5)) { // 5Hz (to match the request above)
          LogControlValues();
        }
      }
    }

    // Lock rate to 25 Hz
    rateLimiter.Limit(25.0);

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

void MikroKopter::GoToLocation(TooN::Vector<2> v2LocInWorld)
{
  std::unique_lock<std::mutex> lock(mMutex);
  cout << "Go to location: " << v2LocInWorld << endl;
  mTargetController.SetTargetLocation(v2LocInWorld);
  mMkConn.SendNewTargetNotice();
}

void MikroKopter::SetTargetAltitude(double altitude)
{
  std::unique_lock<std::mutex> lock(mMutex);
  mTargetController.SetTargetAltitude(altitude);
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
    mMkConn.SendDebugOutputInterval(10);  // Request data every 100ms (10 Hz)
  }
}

void MikroKopter::LogControlValues()
{
  const double* pControl = mTargetController.GetControl();
  mControlValuesFile
    << mTargetController.GetTime() << " "
    << mTargetController.GetTargetOffset()
    << mTargetController.GetVelocity()
    << pControl[0] << " "
    << pControl[1] << " "
    << pControl[2] << " "
    << pControl[3] << " "
    << pControl[4];

  for (size_t i = 0; i < 32; ++i) {
    mControlValuesFile << " " << mMkDebugOutput.Analog[i];
  }

  mControlValuesFile << endl;
}

void MikroKopter::RecvPositionHold()
{
  cout << " >> Position hold requested." << endl;
  mTargetController.HoldCurrentLocation();
}

void MikroKopter::RecvControlRqst(const ControlRequest_t& controlRequest)
{
  mTargetController.SetTargetAltitude((float)controlRequest.altitude * 0.01);
  mTargetController.RequestConfig(controlRequest.status);
}

void MikroKopter::RecvDebugOutput(const DebugOut_t& debugData)
{
  mMkDebugOutput = debugData;
}

}
