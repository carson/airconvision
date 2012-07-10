#include "MikroKopter.h"
#include "Timing.h"
#include "FPSCounter.h"
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

MikroKopter::MikroKopter(const Tracker* pTracker)
  : mpTracker(pTracker)
  , mControllerType(NO_CONTROLLER)
  , mSendDebugTimeout(1.0)
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
  static gvar3<int> gvnOutputWorldCoordinates("Debug.OutputWorldCoordinates", 0, HIDDEN|SILENT);

  std::ofstream coordinateLogFile("coordinates.txt", ios::out | ios::trunc);
  if (!coordinateLogFile) {
    cerr << "Failed to open coordinates.txt" << endl;
  }

  FPSCounter fpsCounter;
  StopWatch stopWatch;
  stopWatch.Start();

  auto t = std::chrono::high_resolution_clock::now();

  while (true) {
    Update(mpTracker->GetCurrentPose(), !mpTracker->IsLost());

    if (*gvnOutputWorldCoordinates) {
      coordinateLogFile << stopWatch.Elapsed() << " " << mpTracker->RealWorldCoordinate() << std::endl;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - t);
    std::this_thread::sleep_for(std::chrono::milliseconds(200 - elapsed.count()));
    t = std::chrono::high_resolution_clock::now();

    if (fpsCounter.Update()) {
      cout << "MK FPS: " << fpsCounter.Fps() << endl;
    }
  }
}

void MikroKopter::Update(const TooN::SE3<> &se3Pose, bool bHasTracking)
{
  using namespace std::chrono;

  // Send world position if connect to MK NaviCtrl
  if (true || mMkConn) {
    //mMkConn.ProcessIncoming();

    if (bHasTracking) {
      switch (mControllerType) {
      case TARGET_CONTROLLER:
        mTargetController.Update(se3Pose, TargetController::Clock::now());
        //mMkConn.SendPositionHoldUpdate(mTargetController.GetTargetOffsetFiltered(),
        //                               mTargetController.GetVelocityFiltered());
        break;
      default:
        break;
      }
    }

    // Check if the debug values from the MK and position hold code should be written to a log
    if (mbWriteControlValuesLog) {
      LogControlValues();
    }

    // Request debug data being sent from the MK, this has to be done every few seconds or
    // the MK will stop sending the data
    /*
    if (mSendDebugTimeout.HasTimedOut()) {
      mMkConn.SendDebugOutputInterval(1);
      mSendDebugTimeout.Reset();
    }
    */
  }
}

void MikroKopter::GoToPosition(const TooN::SE3<> &se3PoseInWorld)
{
  cout << "Go to position: " << se3PoseInWorld.get_translation() << endl;
  mTargetController.SetTarget(se3PoseInWorld);
  mControllerType = TARGET_CONTROLLER;
}

void MikroKopter::AddWaypoint(const TooN::SE3<> &se3PoseInWorld)
{
  cout << "Adding waypoint: " << se3PoseInWorld.get_translation() << endl;
  mPathController.AddWaypoint(se3PoseInWorld);
}

void MikroKopter::ClearWaypoints()
{
  mPathController.ClearWaypoints();
}

void MikroKopter::FlyPath()
{
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
    mMkConn.SetPositionHoldCallback(std::bind(&MikroKopter::RecvRequestPositionHold, this));
    mMkConn.SetDebugOutputCallback(std::bind(&MikroKopter::RecvDebugOutput, this, _1));
    // Request debug data being sent from the MK to this computer
    mMkConn.SendDebugOutputInterval(1);
  }
}

void MikroKopter::LogControlValues()
{
  mControlValuesFile
//    << mTargetController.GetTime() << " "
//    << mTargetController.GetTargetOffset() << " "
    << mTargetController.GetTargetOffsetFiltered() << " "
//    << mTargetController.GetVelocity() << " "
//    << mTargetController.GetVelocityFiltered();
    ;

  for (size_t i = 0; i < 32; ++i) {
    //mControlValuesFile << " " << mMkDebugOutput.Analog[i];
  }

  mControlValuesFile << endl;
}

void MikroKopter::RecvRequestPositionHold()
{
  cout << " >> Position hold requested." << endl;
  /*
  mPositionHold.Init(mpTracker->GetCurrentPose(), high_resolution_clock::now());
  mbPositionHold = true;
  */
}

void MikroKopter::RecvDebugOutput(const DebugOut_t& debug)
{
  mMkDebugOutput = debug;
}

}
