#ifndef __FRONTEND_H
#define __FRONTEND_H

#include "Tracker.h"
#include "InitialTracker.h"
#include "ScaleMarkerTracker.h"
#include "ATANCamera.h"
#include "StereoPlaneFinder.h"
#include "FrameGrabber.h"

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <gvars3/instances.h>

#include <boost/signals2.hpp>

#include <vector>
#include <mutex>
#include <fstream>

namespace PTAMM {

class PerformanceMonitor;

struct FrontendDrawData {

  FrontendDrawData()
    : bInitialTracking(true)
  {
  }

  CVD::Image<CVD::Rgb<CVD::byte>> imFrame;
  CVD::Image<CVD::Rgb<CVD::byte>> imFrameStereo;
  std::string sStatusMessage;

  bool bInitialTracking;
  InitialTrackerDrawData initialTracker;
  TrackerDrawData tracker;

  bool bHasDeterminedScale;
  SE3<> se3MarkerPose;
  TooN::Vector<4> v4GroundPlane;
  TooN::Vector<4> v4DispGroundPlane;

  bool bUseStereo;
  std::vector<CVD::ImageRef> vBackProjectedPts;
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

    void SetCurrentPose(const SE3<> &se3CurrenPose);
    SE3<> GetCurrentPose() const;

  private:
    mutable std::mutex mMutex;

    bool mbHasFrontendData;
    FrontendDrawData mDrawData;

    bool mbUserInvoke;
    bool mbUserResetInvoke;

    SE3<> mse3CurrentPose;
};

class FrameGrabber;
class Tracker;
class FrameData;

class Frontend {
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;
    typedef boost::signals2::signal<void (const SE3<>&, bool, TimePoint)> OnTrackedPoseUpdated;

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

    boost::signals2::connection DoOnTrackedPoseUpdated(const OnTrackedPoseUpdated::slot_type &slot) {
      return mOnTrackedPoseUpdatedSlot.connect(slot);
    }

  private:
    void Reset();
    void ProcessInitialization(bool bUserInvoke);
    void DetermineScaleFromMarker(const FrameData& fd, bool bUserInvoke);

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
    StereoProcessor mStereoProcessor;

    KeyFrame mKeyFrame;

    FrontendDrawData mDrawData;

    SE3<> mse3MarkerPose;
    bool mbSetScaleNextTime;

    GVars3::gvar3<int> mgvnFeatureDetector;

    OnTrackedPoseUpdated mOnTrackedPoseUpdatedSlot;

    std::ofstream mCoordinateLogFile;
};

}


#endif
