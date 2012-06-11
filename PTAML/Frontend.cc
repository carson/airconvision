#include "Frontend.h"
#include "VideoSource.h"
#include "Tracker.h"
#include "Timing.h"
#include "FPSCounter.h"

#include <cvd/image_ref.h>

#include <iostream>

using namespace std;
using namespace CVD;

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

Frontend::Frontend(VideoSource *pVideoSource,
                   const ATANCamera &camera,
                   InitialTracker *pInitialTracker,
                   Tracker *pTracker,
                   ScaleMarkerTracker *pScaleMarkerTracker)
  : mpVideoSource(pVideoSource)
  , mbFreezeVideo(false)
  , mbInitialTracking(true)
  , mbHasDeterminedScale(true)
  , mCamera(camera)
  , mpInitialTracker(pInitialTracker)
  , mpTracker(pTracker)
  , mpScaleMarkerTracker(pScaleMarkerTracker)
{
  ImageRef irVideoSize = mpVideoSource->Size();
  mimFrameBW.resize(irVideoSize);
  mimFrameRGB.resize(irVideoSize);
}

void Frontend::GrabNextFrame()
{
  // Grab new video frame...
  if (!mbFreezeVideo) {
    // We use two versions of each video frame:
    // One black and white (for processing by the tracker etc)
    // and one RGB, for drawing.

    gVideoSourceTimer.Start();
    mpVideoSource->GetAndFillFrameBWandRGB(mimFrameBW, mimFrameRGB);
    gVideoSourceTimer.Stop();
  }
}

void Frontend::operator()()
{
  FPSCounter fpsCounter;

  while (true) {

    GrabNextFrame();

    mDrawData.imFrame.copy_from(mimFrameRGB);
    mDrawData.bInitialTracking = mbInitialTracking;

    if (monitor.PopUserResetInvoke()) {
      // Go back to initial tracking again
      mpInitialTracker->Reset();
      mpTracker->Reset();
      mbInitialTracking = true;
      mbHasDeterminedScale = false;
    }

    gTrackFullTimer.Start();

    if (mbInitialTracking) {
      // Initial tracking path
      if (monitor.PopUserInvoke()) {
        mpInitialTracker->UserInvoke();
      }

      mpInitialTracker->ProcessFrame(mimFrameBW);
      if (mpInitialTracker->IsDone()) {
        mbInitialTracking = false;
        mpTracker->ForceRecovery();
      }

      mDrawData.bInitialTracking = true;
      mDrawData.sStatusMessage = mpInitialTracker->GetMessageForUser();
      mpInitialTracker->GetDrawData(mDrawData.initialTracker);
    } else {
      // Regular map tracking path
      mpTracker->ProcessFrame(mimFrameBW);
      mpTracker->GetDrawData(mDrawData.tracker);
      mDrawData.sStatusMessage = mpTracker->GetMessageForUser();

      if (!mbHasDeterminedScale) {
//        TryDetermineScale();
      }
    }

    monitor.PushDrawData(mDrawData);


    if (fpsCounter.Update()) {
      cout << fpsCounter.Fps() << endl;
    }

    gTrackFullTimer.Stop();
  }
}

void Frontend::TryDetermineScale()
{
  SE3<> se3WorldFromNormWorld;
  double dScale = 1.0;
  if (mpScaleMarkerTracker->DetermineScaleFromMarker(mimFrameBW, mpTracker->GetCurrentPose(),
                                                     se3WorldFromNormWorld, dScale))
  {
    if (monitor.PopUserInvoke()) {
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
