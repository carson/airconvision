#include "Frontend.h"
#include "VideoSource.h"
#include "Tracker.h"
#include "Timing.h"
#include "FPSCounter.h"
#include "FeatureGrid.h"
#include "StereoInitializer.h"

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
                   MapMaker *pMapMaker,
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
  , mpMapMaker(pMapMaker)
  , mCurrentKF(camera)
  , mStereoKF(camera)
{
  GV3::Register(mgvnFeatureDetector, "FeatureDetector", (int)PLAIN_FAST10, SILENT);

  ImageRef irVideoSize = mpVideoSource->Size();
  mimFrameBW.resize(irVideoSize);
  mimFrameRGB.resize(irVideoSize);
  mimStereoFrameBW.resize(irVideoSize);
}

void Frontend::GrabNextFrame()
{
  // Grab new video frame...
  if (!mbFreezeVideo) {
    // We use two versions of each video frame:
    // One black and white (for processing by the tracker etc)
    // and one RGB, for drawing.

    gVideoSourceTimer.Start();

    CVD::img_load(mimFrameRGB, "first.bmp");
    convert_image(mimFrameRGB, mimFrameBW);

//    mpVideoSource->GetAndFillFrameBWandRGB(mimFrameBW, mimFrameRGB);

    gVideoSourceTimer.Stop();
  }
}

void Frontend::operator()()
{
  static gvar3<int> gvnStereoInit("StereoInit", 1, SILENT);

  FPSCounter fpsCounter;

  while (true) {

    bool bUserInvoke = monitor.PopUserInvoke();
    bool bUserResetInvoke = monitor.PopUserResetInvoke();

    GrabNextFrame();

    mCurrentKF.InitFromImage(mimFrameBW, (FeatureDetector)*mgvnFeatureDetector);

    mDrawData.imFrame.copy_from(mimFrameRGB);
    mDrawData.bInitialTracking = mbInitialTracking;

    if (bUserResetInvoke) {
      // Go back to initial tracking again
      mpInitialTracker->Reset();
      mpTracker->Reset();
      mCurrentKF.Reset();
      mbInitialTracking = true;
      mbHasDeterminedScale = false;
    }

    gTrackFullTimer.Start();

    if (mbInitialTracking) {
      if (*gvnStereoInit) {

        if (bUserInvoke) {
          InitFromStereo();
        }

        mDrawData.bInitialTracking = true;
        mDrawData.sStatusMessage = "Press spacebar to init";

      } else {
        // Initial tracking path
        if (bUserInvoke) {
          mpInitialTracker->UserInvoke();
        }

        mpInitialTracker->ProcessFrame(mCurrentKF);
        if (mpInitialTracker->IsDone()) {
          mpTracker->SetCurrentPose(mpInitialTracker->GetCurrentPose());
          mbInitialTracking = false;
          mpTracker->ForceRecovery();
        }

        mDrawData.bInitialTracking = true;
        mDrawData.sStatusMessage = mpInitialTracker->GetMessageForUser();
        mpInitialTracker->GetDrawData(mDrawData.initialTracker);
      }
    } else {
      // Regular map tracking path
      mpTracker->ProcessFrame(mCurrentKF);
      mpTracker->GetDrawData(mDrawData.tracker);
      mDrawData.sStatusMessage = mpTracker->GetMessageForUser();

      if (!mbHasDeterminedScale) {
//        TryDetermineScale();
      }
    }

    monitor.PushDrawData(mDrawData);


    if (fpsCounter.Update()) {
      //cout << fpsCounter.Fps() << endl;
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

void Frontend::InitFromStereo()
{
//  mpVideoSource2->GetAndFillFrameBWandRGB(imFrameBW2, imFrameRGB2);

  CVD::Image<CVD::Rgb<CVD::byte> > tmp;
  CVD::img_load(tmp, "second.bmp");
  convert_image(tmp, mimStereoFrameBW);


  mStereoKF.InitFromImage(mimStereoFrameBW, (FeatureDetector)*mgvnFeatureDetector);

  cout << mimStereoFrameBW.size() << endl;

  SE3<> se3SecondCameraPos; // = GV3::get<SE3<>>("SecondCameraPos", SE3<>(), SILENT);


  Matrix<3> r;
  r[0] = makeVector( 9.9919373211873597e-01, -1.7221945311748101e-03, -4.0111341795413702e-02);
  r[1] = makeVector( 3.9772309517540523e-03, 9.9841117628130216e-01, 5.6207692627815442e-02);
  r[2] = makeVector( 3.9950811363326774e-02, -5.6321906240674777e-02, 9.9761303898296594e-01);
  Vector<3> t = makeVector( -1.1837034695051358e+01, -1.6813272577166739e-02, -1.1040735338436671e+00 );

  se3SecondCameraPos = SE3<>(r, t);

  cout << se3SecondCameraPos << endl;

  mpMapMaker->InitFromStereo(mCurrentKF, mStereoKF, se3SecondCameraPos);

  mbInitialTracking = false;
}

}
