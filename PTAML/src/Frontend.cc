#include "Frontend.h"
#include "VideoSource.h"
#include "Tracker.h"
#include "PerformanceMonitor.h"
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
                   ScaleMarkerTracker *pScaleMarkerTracker,
                   PerformanceMonitor *pPerfMon)
  : mbDone(false)
  , mbInitialTracking(true)
  , mbHasDeterminedScale(false)
  , mCamera(camera)
  , mpFrameGrabber(pFrameGrabber)
  , mpInitialTracker(pInitialTracker)
  , mpTracker(pTracker)
  , mpScaleMarkerTracker(pScaleMarkerTracker)
  , mpMapMaker(pMapMaker)
  , mpPerfMon(pPerfMon)
  , mKeyFrame(camera)
  , mbSetScaleNextTime(false)
{
  GV3::Register(mgvnFeatureDetector, "FeatureDetector", (int)PLAIN_FAST10, SILENT);
}

void Frontend::operator()()
{
  while (!mbDone) {
    bool bUserInvoke = monitor.PopUserInvoke();
    bool bUserResetInvoke = monitor.PopUserResetInvoke();

    mpFrameGrabber->GrabNextFrame();

    // Initialize keyframe, find features etc
    mKeyFrame.InitFromImage(mpFrameGrabber->GetFrameBW1(),
                            static_cast<FeatureDetector>(*mgvnFeatureDetector));

    // Set some of the draw data
    mDrawData.imFrame.copy_from(mpFrameGrabber->GetFrameRGB1());
    mDrawData.bInitialTracking = mbInitialTracking;

    if (bUserResetInvoke) {
      // Go back to initial tracking again
      Reset();
    }

    mpPerfMon->StartTimer("tracking_total");

    if (mbInitialTracking) {
      ProcessInitialization(bUserInvoke);
    }

    bool bRunTracker = !mbInitialTracking;
    mpTracker->ProcessFrame(mKeyFrame, bRunTracker);

    if (bRunTracker) {
      // Regular map tracking path

      if (!mbHasDeterminedScale) {
        DetermineScaleFromMarker(bUserInvoke);
      }

      mpTracker->GetDrawData(mDrawData.tracker);
      mDrawData.bHasDeterminedScale = mbHasDeterminedScale;
      mDrawData.se3MarkerPose = mse3MarkerPose;
      mDrawData.sStatusMessage = mpTracker->GetMessageForUser();
      mDrawData.bInitialTracking = false;
    }

    monitor.PushDrawData(mDrawData);

    mpPerfMon->UpdateRateCounter("frontend");

    mpPerfMon->StopTimer("tracking_total");
  }
}

void Frontend::Reset()
{
  mpInitialTracker->Reset();
  mpTracker->Reset();
  mKeyFrame.Reset();
  mbInitialTracking = true;
  mbHasDeterminedScale = false;
  mbSetScaleNextTime = false;
}

void Frontend::ProcessInitialization(bool bUserInvoke)
{
  if (mpFrameGrabber->IsUsingStereo()) {

    mbHasDeterminedScale = true;

    // Calculate a dense point cloud
    mpFrameGrabber->ProcessStereoImages();

    // Find the ground plane in the point cloud
    mStereoPlaneFinder.Update(mpFrameGrabber->GetPointCloud());

    if (bUserInvoke) {
      SE3<> se3CurrentPose;
      mpMapMaker->InitFromKnownPlane(mKeyFrame, mStereoPlaneFinder.GetPlane(), se3CurrentPose);
      mpTracker->SetCurrentPose(se3CurrentPose);
      //mpTracker->ForceRecovery();
      mbInitialTracking = false;
    }

    mDrawData.bUseStereo = true;
    mDrawData.bInitialTracking = true;
    mDrawData.sStatusMessage = "Press spacebar to init";
    mDrawData.v4GroundPlane = mStereoPlaneFinder.GetPlane();

  } else {
    // Initial tracking path
    if (bUserInvoke) {
      mpInitialTracker->UserInvoke();
    }

    mpInitialTracker->ProcessFrame(mKeyFrame);
    if (mpInitialTracker->IsDone()) {
      mpTracker->SetCurrentPose(mpInitialTracker->GetCurrentPose());
      mbInitialTracking = false;

      cout << "Inited to pose: " << mpInitialTracker->GetCurrentPose() << endl;
    }

    mDrawData.bUseStereo = false;
    mDrawData.bInitialTracking = true;
    mDrawData.sStatusMessage = mpInitialTracker->GetMessageForUser();
    mpInitialTracker->GetDrawData(mDrawData.initialTracker);
  }
}

void Frontend::DetermineScaleFromMarker(bool bUserInvoke)
{
  mbSetScaleNextTime = mbSetScaleNextTime || bUserInvoke;

  SE3<> se3WorldFromNormWorld;
  double dScale = 1.0;
  if (mpScaleMarkerTracker->DetermineScaleFromMarker(mpFrameGrabber->GetFrameBW1(),
                                                     mpTracker->GetCurrentPose(),
                                                     se3WorldFromNormWorld, dScale))
  {
    cout << "SCALE: " << dScale << endl;
    mse3MarkerPose = se3WorldFromNormWorld;


    if (mbSetScaleNextTime) {

      mpMapMaker->TransformMapPoints(mse3MarkerPose.inverse());
      mpMapMaker->ScaleMapPoints(dScale);

      SE3<> se3CamFromWorld = mse3MarkerPose.inverse() * mpTracker->GetCurrentPose();
      se3CamFromWorld.get_translation() *= dScale;
      mpTracker->SetCurrentPose(se3CamFromWorld);

      mbHasDeterminedScale = true;

      cout << "Scale is set! " <<  dScale << endl;
    }
  }


}

}
