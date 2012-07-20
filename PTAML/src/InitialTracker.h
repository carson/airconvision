#ifndef __INITIAL_TRACKER_H
#define __INITIAL_TRACKER_H

#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"

#include <sstream>
#include <vector>

namespace PTAMM {

// This struct is used for initial correspondences of the first stereo pair.
struct Trail {
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos;
  CVD::ImageRef irInitialPos;
};

struct InitialTrackerDrawData {
  std::vector<std::pair<CVD::ImageRef, CVD::ImageRef>> vTrails;
  std::vector<CVD::ImageRef> vDeadTrails;
  std::vector<CVD::ImageRef> vCorners;
};

class InitialTracker {
  public:
    InitialTracker(const CVD::ImageRef &irVideoSize, const ATANCamera &c, Map *m, MapMaker *mm);

    // TrackFrame is the main working part of the tracker: call this every frame.
    void ProcessFrame(KeyFrame &keyFrame);
    void GetDrawData(InitialTrackerDrawData &drawData);
    void Reset();

    bool IsDone() const { return mStage == TRAIL_TRACKING_COMPLETE; }
    const SE3<>& GetCurrentPose() const{ return mse3CamFromWorld; }

    void UserInvoke() { mbUserPressedSpacebar = true; }
    std::string GetMessageForUser() const;

  private:
    // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
    void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.
    void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
    int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.

  private:
    enum InitialTrackingStage {
      TRAIL_TRACKING_NOT_STARTED,
      TRAIL_TRACKING_STARTED,
      WAITING_FOR_STEREO_INIT,
      TRAIL_TRACKING_COMPLETE
    };

    // The major components to which the tracker needs access:
    Map *mpMap;                     // The map, consisting of points and keyframes
    MapMaker *mpMapMaker;           // The class which maintains the map
    ATANCamera mCamera;             // Projection model

    CVD::ImageRef mirSize;          // Image size of whole image

    InitialTrackingStage mStage;    // How far are we towards making the initial map?

    KeyFrame *mpCurrentKF;            // The current working frame as a keyframe struct
    KeyFrame mFirstKF;              // First of the stereo pair
    KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches
    SE3<> mse3CamFromWorld;         // Camera pose: this is what the tracker updates every frame.

    std::vector<Trail> mlTrails;    // Used by trail tracking
    std::vector<CVD::ImageRef> mvDeadTrails;

    // User interaction for initial tracking:
    bool mbUserPressedSpacebar;
    std::ostringstream mMessageForUser;
};

}

#endif
