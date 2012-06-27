#ifndef __FRONTEND_H
#define __FRONTEND_H

#include "Tracker.h"
#include "InitialTracker.h"
#include "ScaleMarkerTracker.h"
#include "ATANCamera.h"

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <gvars3/instances.h>

#include <vector>
#include <mutex>

namespace PTAMM {

struct FrontendDrawData {

  FrontendDrawData()
    : bInitialTracking(true)
  {
  }

  CVD::Image<CVD::Rgb<CVD::byte>> imFrame;
  std::string sStatusMessage;

  bool bInitialTracking;
  InitialTrackerDrawData initialTracker;
  TrackerDrawData tracker;
};

class FrontendMonitor {
  public:
    FrontendMonitor()
      : mbHasFrontendData(false)
      , mbUserInvoke(false)
      , mbUserResetInvoke(false)
    {
    }

    void PushDrawData(FrontendDrawData &drawData);
    bool PopDrawData(FrontendDrawData &drawData);

    void PushUserInvoke();
    bool PopUserInvoke();

    void PushUserResetInvoke();
    bool PopUserResetInvoke();

  private:
    std::mutex mMutex;

    bool mbHasFrontendData;
    FrontendDrawData mDrawData;

    bool mbUserInvoke;
    bool mbUserResetInvoke;
};

class VideoSource;
class Tracker;

class Frontend {
  public:
    Frontend(VideoSource *pVideoSource,
             const ATANCamera &camera,
             MapMaker *pMapMaker,
             InitialTracker *pInitialTracker,
             Tracker *pTracker,
             ScaleMarkerTracker *pScaleMarkerTracker);

    void operator()();

    FrontendMonitor monitor;

  private:
    void GrabNextFrame();
    void TryDetermineScale();
    void InitFromStereo();

  private:
    VideoSource *mpVideoSource;
    VideoSource *mpVideoSource2;
    CVD::Image<CVD::Rgb<CVD::byte>> mimFrameRGB;   // The RGB image used for AR
    CVD::Image<CVD::byte> mimFrameBW;               // The Black and white image for tracking/mapping
    CVD::Image<CVD::byte> mimStereoFrameBW;
    bool mbFreezeVideo;

    bool mbInitialTracking;
    bool mbHasDeterminedScale;

    ATANCamera mCamera;
    InitialTracker *mpInitialTracker;
    Tracker *mpTracker;
    ScaleMarkerTracker *mpScaleMarkerTracker;
    MapMaker *mpMapMaker;

    KeyFrame mCurrentKF;
    KeyFrame mStereoKF;

    FrontendDrawData mDrawData;

    GVars3::gvar3<int> mgvnFeatureDetector;
};

}


#endif
