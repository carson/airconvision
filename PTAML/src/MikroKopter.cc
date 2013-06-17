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
  , mbUpdateReady(false)
  , mbLogMKControl(false)
  , mbLogMKData(false)
  , mbLogMKDebug(false)
  , mMKDebugRequestTimeout(2.0)
  , mMKData()  // zeros the structure
  , mMKDebug()  // zeros the structure
  , mMKToPTAM()  // zeros the structure
{
  // Read COM port settings
  int nComPortId = GV3::get<int>("MKNaviCtrl.ComPortId", 16, SILENT); // 16 is /dev/ttyUSB0
  int nCcomPortBaudrate = GV3::get<int>("MKNaviCtrl.ComPortBaudrate", "57600", SILENT);
  ConnectToMK(nComPortId, nCcomPortBaudrate);

  // Log the control data
  mbLogMKControl = GV3::get<int>("Debug.LogMKControl", 0, SILENT);
  if (mbLogMKControl) {
    mMKControlLogFile.open("mk_control.txt", ios::out | ios::trunc);
    if (!mMKControlLogFile) {
      cerr << "Failed to open mk_control.txt" << endl;
    }
  }
  // Log the data stream from the MK
  mbLogMKData = GV3::get<int>("Debug.LogMKData", 0, SILENT);
  if (mbLogMKData) {
    mMKDataLogFile.open("mk_data.txt", ios::out | ios::trunc);
    if (!mMKDataLogFile) {
      cerr << "Failed to open mk_data.txt" << endl;
    }
  }

  // Log the debug output stream from the MK
  mbLogMKDebug = GV3::get<int>("Debug.LogMKDebug", 0, SILENT);
  if (mbLogMKDebug) {
    mMKDebugLogFile.open("mk_debug.txt", ios::out | ios::trunc);
    if (!mMKDebugLogFile) {
      cerr << "Failed to open mk_debug.txt" << endl;
    }
  }
}

void MikroKopter::operator()()
{
  RateLimiter rateLimiter;
  static int counter = 0;

  while (!mbDone) {
    if (mMkConn) {
      mMkConn.ProcessIncoming();

      // Don't try to send two commands on the same frame
      if (mbLogMKDebug && mMKDebugRequestTimeout.HasTimedOut()) {
        // Request debug data being sent from the MK, this has to be done every few seconds or
        // the MK will stop sending the data
        mMkConn.RequestMKDebugInterval(10);  // Request data every 100ms (10 Hz)
        mMKDebugRequestTimeout.Reset();
      } else if (mbUpdateReady) {
        std::unique_lock<std::mutex> lock(mMutex);

        mMkConn.SendPTAMToMK(mTargetController.GetControl(),
            mTargetController.GetEulerAngles(), mTargetController.GetConfig());

        mbUpdateReady = false;
      }
    }

    // Lock rate to 256 Hz
    rateLimiter.Limit(256.0);

    mpPerfMon->UpdateRateCounter("mk");
    counter++;
  }
}

void MikroKopter::UpdatePose(const TooN::SE3<> &se3Pose, bool bHasTracking)
{
  std::unique_lock<std::mutex> lock(mMutex);
  mbHasTracking = bHasTracking;
  mTargetController.Update(se3Pose, bHasTracking, TargetController::Clock::now());
  mbUpdateReady = true;
  if (mbLogMKControl) LogMKControl();
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
    mMkConn.SetMKToPTAMCallback(std::bind(&MikroKopter::RecvMKToPTAM, this, _1));
    mMkConn.SetMKDebugCallback(std::bind(&MikroKopter::RecvMKDebug, this, _1));
  }
}

void MikroKopter::RecvPositionHold()
{
  cout << " >> Position hold requested." << endl;
  mTargetController.HoldCurrentLocation();
}

void MikroKopter::RecvMKToPTAM(const MKToPTAM_t& mkToPTAM)
{
  mTargetController.SetTargetAltitude((float)mkToPTAM.altitude * 0.01);
  mTargetController.RequestConfig(mkToPTAM.request);
  mMKToPTAM = mkToPTAM;
  if (mbLogMKData) LogMKData();
}

void MikroKopter::RecvMKDebug(const MKDebug_t& mkDebug)
{
  mMKDebug = mkDebug;
}

void MikroKopter::LogMKControl()
{
  mMKControlLogFile
      << mTargetController.GetTime()
      << " " << mTargetController.GetPosInWorld()
      << mTargetController.GetTarget()
      << mTargetController.GetTargetOffset()
      << mTargetController.GetEulerAngles()
      << mTargetController.GetVelocity()
      << (int)mTargetController.GetConfig()
      << " " << (int)mTargetController.GetExperimentMode();
  for (size_t i = 0; i < 5; ++i) {
    mMKControlLogFile << " " << mTargetController.GetControl()[i];
  }
  mMKControlLogFile << endl;
}

void MikroKopter::LogMKData()
{
  mMKDataLogFile
      << (int16_t)mMKToPTAM.count
      << " " << (int16_t)mMKToPTAM.request
      << " " << ((float)mMKToPTAM.altitude * 0.01);

  for (size_t i = 0; i < 6; ++i) {
    mMKDataLogFile << " " << mMKToPTAM.int16[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    mMKDataLogFile << " " << mMKToPTAM.single[i];
  }
  mMKDataLogFile << endl;
}

void MikroKopter::LogMKDebug()
{
  for (size_t i = 0; i < 32; ++i) {
    mMKDebugLogFile << " " << mMKDebug.int16[i];
  }

  mMKDebugLogFile << endl;
}

}
