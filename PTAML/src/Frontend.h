#ifndef __FRONTEND_H
#define __FRONTEND_H

#include "Tracker.h"
#include "InitialTracker.h"
#include "ScaleMarkerTracker.h"
#include "ATANCamera.h"
#include "StereoPlaneFinder.h"

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <gvars3/instances.h>

#include <vector>
#include <mutex>

namespace PTAMM {

class PerformanceMonitor;

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

  bool bHasDeterminedScale;
  SE3<> se3MarkerPose;
  TooN::Vector<4> v4GroundPlane;

  bool bUseStereo;
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

class FrameGrabber;
class Tracker;
class FrameData;

class Frontend {
  public:
    Frontend(FrameGrabber *pFrameGrabber,
             const ATANCamera &camera,
             MapMaker *pMapMaker,
             InitialTracker *pInitialTracker,
             Tracker *pTracker,
             ScaleMarkerTracker *pScaleMarkerTracker,
             PerformanceMonitor *pPerfMon);

    void operator()();

    void StopThread() { mbDone = true; }

    FrontendMonitor monitor;

  private:
    void Reset();
    void ProcessInitialization(bool bUserInvoke);
    void DetermineScaleFromMarker(const FrameData& fd, bool bUserInvoke);

  private:
    bool mbDone;

    bool mbInitialTracking;
    bool mbHasDeterminedScale;

    ATANCamera mCamera;
    FrameGrabber *mpFrameGrabber;
    InitialTracker *mpInitialTracker;
    Tracker *mpTracker;
    ScaleMarkerTracker *mpScaleMarkerTracker;
    MapMaker *mpMapMaker;
    StereoPlaneFinder mStereoPlaneFinder;
    PerformanceMonitor *mpPerfMon;

    KeyFrame mKeyFrame;

    FrontendDrawData mDrawData;

    SE3<> mse3MarkerPose;
    bool mbSetScaleNextTime;

    GVars3::gvar3<int> mgvnFeatureDetector;
};

}


#endif
