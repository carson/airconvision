#include "Frontend.h"
#include "VideoSource.h"
#include "Tracker.h"
#include "Timing.h"
#include "FPSCounter.h"
#include "FeatureGrid.h"
#include "FrameGrabber.h"

#include <cvd/image_ref.h>
#include <cvd/image_io.h>

#include <iostream>

using namespace std;
using namespace CVD;
using namespace GVars3;

namespace PTAMM {

void FrontendMonitor::PushDrawData(FrontendDrawData &drawData)
{
  std::lock_guard<std::mutex> lock(mMutex);
  using std::swap;
  swap(mDrawData, drawData);
  mbHasFrontendData = true;
}

bool FrontendMonitor::PopDrawData(FrontendDrawData &drawData)
{
  std::lock_guard<std::mutex> lock(mMutex);
  if (!mbHasFrontendData) {
    return false;
  }

  using std::swap;
  swap(mDrawData, drawData);
  mbHasFrontendData = false;
  return true;
}

void FrontendMonitor::PushUserInvoke()
{
  std::lock_guard<std::mutex> lock(mMutex);
  mbUserInvoke = true;
}

bool FrontendMonitor::PopUserInvoke()
{
  std::lock_guard<std::mutex> lock(mMutex);
  if (mbUserInvoke) {
    mbUserInvoke = false;
    return true;
  }

  return false;
}

void FrontendMonitor::PushUserResetInvoke()
{
  std::lock_guard<std::mutex> lock(mMutex);
  mbUserResetInvoke = true;
}

bool FrontendMonitor::PopUserResetInvoke()
{
  std::lock_guard<std::mutex> lock(mMutex);
  if (mbUserResetInvoke) {
    mbUserResetInvoke = false;
    return true;
  }

  return false;
}

Frontend::Frontend(FrameGrabber *pFrameGrabber,
                   const ATANCamera &camera,
                   MapMaker *pMapMaker,
                   InitialTracker *pInitialTracker,
                   Tracker *pTracker,
                   ScaleMarkerTracker *pScaleMarkerTracker)
  : mbInitialTracking(true)
  , mbHasDeterminedScale(true)
  , mCamera(camera)
  , mpFrameGrabber(pFrameGrabber)
  , mpInitialTracker(pInitialTracker)
  , mpTracker(pTracker)
  , mpScaleMarkerTracker(pScaleMarkerTracker)
  , mpMapMaker(pMapMaker)
  , mKeyFrame(camera)
{
  GV3::Register(mgvnFeatureDetector, "FeatureDetector", (int)PLAIN_FAST10, SILENT);
}

void Frontend::operator()()
{
  FPSCounter fpsCounter;

  while (true) {

    bool bUserInvoke = monitor.PopUserInvoke();
    bool bUserResetInvoke = monitor.PopUserResetInvoke();

    mpFrameGrabber->GrabNextFrame();

    // Initialize keyframe, find features etc
    mKeyFrame.InitFromImage(mpFrameGrabber->GetFrameBW1(), (FeatureDetector)*mgvnFeatureDetector);

    // Set some of the draw data
    mDrawData.imFrame.copy_from(mpFrameGrabber->GetFrameRGB1());
    mDrawData.bInitialTracking = mbInitialTracking;

    if (bUserResetInvoke) {
      // Go back to initial tracking again
      Reset();
    }

    gTrackFullTimer.Start();

    if (mbInitialTracking) {
      ProcessInitialization(bUserInvoke);
    } else {
      // Regular map tracking path
      mpTracker->ProcessFrame(mKeyFrame);
      mpTracker->GetDrawData(mDrawData.tracker);
      mDrawData.sStatusMessage = mpTracker->GetMessageForUser();

      if (!mbHasDeterminedScale) {
        DetermineScaleFromMarker(bUserInvoke);
      }
    }

    monitor.PushDrawData(mDrawData);

    if (fpsCounter.Update()) {
      //cout << fpsCounter.Fps() << endl;
    }

    gTrackFullTimer.Stop();
  }
}

void Frontend::Reset()
{
  mpInitialTracker->Reset();
  mpTracker->Reset();
  mKeyFrame.Reset();
  mbInitialTracking = true;
  mbHasDeterminedScale = false;
}

void Frontend::ProcessInitialization(bool bUserInvoke)
{
  if (mpFrameGrabber->IsUsingStereo()) {
    //mStereoPlaneTracker.Update();

    if (bUserInvoke) {
      //mpMapMaker->InitFromKnownPlane(mKeyFrame, mStereoPlaneTracker.GetPlane());
      mbInitialTracking = false;
    }

    mDrawData.bInitialTracking = true;
    mDrawData.sStatusMessage = "Press spacebar to init";

  } else {
    // Initial tracking path
    if (bUserInvoke) {
      mpInitialTracker->UserInvoke();
    }

    mpInitialTracker->ProcessFrame(mKeyFrame);
    if (mpInitialTracker->IsDone()) {
      mpTracker->SetCurrentPose(mpInitialTracker->GetCurrentPose());
      mbInitialTracking = false;
      mpTracker->ForceRecovery();
    }

    mDrawData.bInitialTracking = true;
    mDrawData.sStatusMessage = mpInitialTracker->GetMessageForUser();
    mpInitialTracker->GetDrawData(mDrawData.initialTracker);
  }
}

void Frontend::DetermineScaleFromMarker(bool bUserInvoke)
{
  SE3<> se3WorldFromNormWorld;
  double dScale = 1.0;
  if (mpScaleMarkerTracker->DetermineScaleFromMarker(mpFrameGrabber->GetFrameBW1(),
                                                     mpTracker->GetCurrentPose(),
                                                     se3WorldFromNormWorld, dScale))
  {
    if (bUserInvoke) {
      cout << "SCALE: " << dScale << endl;
      //mMapMaker.RequestMapTransformation(se3WorldFromNormWorld.inverse());
      //mMapMaker.RequestMapScaling(dScale);

      /*
      mMapMaker.RequestCallback([&] () {
        mse3CamFromWorld = se3WorldFromNormWorld * mse3CamFromWorld;
        mse3CamFromWorld.get_translation() *= scale;
        mbFreezeTracking = false;
        ForceRecovery();
      } );
      */

      mbHasDeterminedScale = true;
    }
  }
}

}
